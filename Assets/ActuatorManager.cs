// File: Assets/Scripts/Boilerplate/ActuatorManager.cs
// This script manages all actuators on the robot, translating high-level commands
// into low-level control signals for motors, grippers, and other movable parts.
// It can include PID controllers for precise movement and feedback.

using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq; // For LINQ operations

namespace RobotBoilerplate
{
    /// <summary>
    /// Base class for all actuator configurations.
    /// </summary>
    [Serializable]
    public class ActuatorConfig
    {
        public ActuatorType type;       // Type of actuator.
        public int id;                  // Unique ID for this actuator instance.
        public string name;             // Human-readable name.
        public bool isEnabled;          // Whether this actuator is active.
        public float maxSpeed;          // Maximum speed/velocity of the actuator.
        public float maxForceTorque;    // Maximum force/torque the actuator can exert.

        public ActuatorConfig(ActuatorType type, int id, string name, bool enabled = true, float maxSpeed = 1f, float maxForceTorque = 10f)
        {
            this.type = type;
            this.id = id;
            this.name = name;
            this.isEnabled = enabled;
            this.maxSpeed = maxSpeed;
            this.maxForceTorque = maxForceTorque;
        }

        public override string ToString()
        {
            return $"{name} ({type} ID:{id}) - Enabled: {isEnabled}";
        }
    }

    /// <summary>
    /// Configuration for a Drive Motor actuator.
    /// </summary>
    [Serializable]
    public class DriveMotorConfig : ActuatorConfig
    {
        public float wheelRadius;       // Radius of the wheel attached to this motor.
        public float gearRatio;         // Gear ratio from motor to wheel.

        public DriveMotorConfig(int id, string name, float wheelR = 0.1f, float gearR = 10f)
            : base(ActuatorType.DriveMotor, id, name, true, 5f, 50f)
        {
            wheelRadius = wheelR;
            gearRatio = gearR;
        }
    }

    /// <summary>
    /// Configuration for an Arm Joint actuator.
    /// </summary>
    [Serializable]
    public class ArmJointConfig : ActuatorConfig
    {
        public float minAngle;          // Minimum angle for the joint (degrees).
        public float maxAngle;          // Maximum angle for the joint (degrees).
        public float currentAngle;      // Current angle of the joint (for inspection).

        public ArmJointConfig(int id, string name, float minA = -90f, float maxA = 90f)
            : base(ActuatorType.ArmJoint, id, name, true, 60f, 20f) // Max speed in deg/s
        {
            minAngle = minA;
            maxAngle = maxA;
            currentAngle = 0f;
        }
    }

    /// <summary>
    /// Configuration for a Gripper actuator.
    /// </summary>
    [Serializable]
    public class GripperConfig : ActuatorConfig
    {
        public float openPosition;      // Value representing the fully open position.
        public float closedPosition;    // Value representing the fully closed position.
        public float gripForce;         // Force exerted when gripping.

        public GripperConfig(int id, string name)
            : base(ActuatorType.Gripper, id, name, true, 0.5f, 10f) // Max speed in m/s, max force in N
        {
            openPosition = 1.0f;
            closedPosition = 0.0f;
            gripForce = 50f;
        }
    }

    /// <summary>
    /// A simple PID controller for precise actuator control.
    /// </summary>
    [Serializable]
    public class PIDController
    {
        public float Kp; // Proportional gain
        public float Ki; // Integral gain
        public float Kd; // Derivative gain

        private float _integral;
        private float _lastError;

        public PIDController(float p, float i, float d)
        {
            Kp = p;
            Ki = i;
            Kd = d;
            _integral = 0;
            _lastError = 0;
        }

        /// <summary>
        /// Calculates the control output based on the error.
        /// </summary>
        /// <param name="error">The difference between target and current value.</param>
        /// <param name="deltaTime">The time elapsed since the last update.</param>
        /// <returns>The calculated control output.</returns>
        public float Calculate(float error, float deltaTime)
        {
            _integral += error * deltaTime;
            float derivative = (error - _lastError) / deltaTime;
            _lastError = error;

            return Kp * error + Ki * _integral + Kd * derivative;
        }

        /// <summary>
        /// Resets the integral and derivative terms.
        /// </summary>
        public void Reset()
        {
            _integral = 0;
            _lastError = 0;
        }
    }

    /// <summary>
    /// Manages all actuators on the robot, providing methods to command them
    /// and potentially implementing low-level control loops (e.g., PID).
    /// </summary>
    public class ActuatorManager : MonoBehaviour
    {
        [Header("Actuator Configuration")]
        [Tooltip("List of all actuators configured for this robot.")]
        [SerializeReference] // Allows serialization of derived classes
        private List<ActuatorConfig> _actuatorConfigurations = new List<ActuatorConfig>();

        [Header("Actuator References")]
        [Tooltip("References to the actual Unity GameObjects/Components representing actuators.")]
        [SerializeField] private List<GameObject> _driveMotorObjects = new List<GameObject>();
        [SerializeField] private List<GameObject> _armJointObjects = new List<GameObject>();
        [SerializeField] private GameObject _gripperObject;

        [Header("PID Controllers")]
        [Tooltip("PID controllers for drive motors (indexed by motor ID).")]
        private Dictionary<int, PIDController> _driveMotorPIDs = new Dictionary<int, PIDController>();
        [Tooltip("PID controllers for arm joints (indexed by joint ID).")]
        private Dictionary<int, PIDController> _armJointPIDs = new Dictionary<int, PIDController>();

        // Private fields for internal management
        private Dictionary<int, float> _currentMotorVelocities = new Dictionary<int, float>(); // Current reported velocity
        private Dictionary<int, float> _currentJointAngles = new Dictionary<int, float>(); // Current reported angle
        private bool _isInitialized = false;

        // --- Event System ---
        public event Action<MotorCommand> OnMotorCommandSent;
        public event Action<int, float> OnJointCommandSent;
        public event Action<bool> OnGripperStateChanged;
        public event Action<string> OnActuatorError;

        /// <summary>
        /// Called when the script instance is being loaded.
        /// </summary>
        private void Awake()
        {
            Debug.Log("ActuatorManager: Awake - Initializing.");
            InitializeActuators();
            _isInitialized = true;
        }

        /// <summary>
        /// Called on the frame when a script is enabled just before any of the Update methods are called the first time.
        /// </summary>
        private void Start()
        {
            if (!_isInitialized) return;
            Debug.Log("ActuatorManager: Start - Actuator control ready.");
        }

        /// <summary>
        /// FixedUpdate is called at a fixed framerate, independent of frame rate.
        /// Used for physics calculations and applying forces.
        /// </summary>
        private void FixedUpdate()
        {
            // Run PID loops for motors/joints if enabled and necessary
            // This would typically involve reading sensor feedback and applying forces.
            RunMotorPIDLoops();
            RunArmJointPIDLoops();
        }

        /// <summary>
        /// Initializes all configured actuators and their associated PID controllers.
        /// </summary>
        private void InitializeActuators()
        {
            _driveMotorPIDs.Clear();
            _armJointPIDs.Clear();
            _currentMotorVelocities.Clear();
            _currentJointAngles.Clear();

            if (_actuatorConfigurations == null || _actuatorConfigurations.Count == 0)
            {
                Debug.LogWarning("ActuatorManager: No actuator configurations found. Adding default drive motors and gripper.");
                // Add some default configurations if none exist
                _actuatorConfigurations.Add(new DriveMotorConfig(0, "LeftDriveMotor"));
                _actuatorConfigurations.Add(new DriveMotorConfig(1, "RightDriveMotor"));
                _actuatorConfigurations.Add(new GripperConfig(2, "MainGripper"));
                _actuatorConfigurations.Add(new ArmJointConfig(3, "ArmBaseJoint"));
            }

            foreach (var config in _actuatorConfigurations)
            {
                if (!config.isEnabled)
                {
                    Debug.Log($"Actuator {config.name} (ID:{config.id}) is disabled.");
                    continue;
                }

                Debug.Log($"ActuatorManager: Initializing {config.name} (ID:{config.id}, Type:{config.type}).");

                switch (config.type)
                {
                    case ActuatorType.DriveMotor:
                        // Initialize PID for each motor
                        _driveMotorPIDs[config.id] = new PIDController(50f, 0.1f, 1f); // Example PID gains
                        _currentMotorVelocities[config.id] = 0f;
                        // Placeholder: Find and link actual motor components in Unity scene
                        // e.g., _driveMotorObjects[config.id].GetComponent<JointMotor>();
                        break;
                    case ActuatorType.ArmJoint:
                        _armJointPIDs[config.id] = new PIDController(10f, 0.05f, 0.5f); // Example PID gains
                        _currentJointAngles[config.id] = 0f;
                        // Placeholder: Find and link actual joint components
                        break;
                    case ActuatorType.Gripper:
                        // Gripper might not need a PID, just direct open/close commands
                        break;
                    case ActuatorType.PanTiltUnit:
                    case ActuatorType.LinearActuator:
                        // Add specific initialization for other actuator types
                        break;
                }
            }
            Debug.Log("ActuatorManager: Actuator initialization complete.");
        }

        /// <summary>
        /// Runs PID control loops for all configured drive motors.
        /// This would read current motor velocity feedback and apply forces/torques.
        /// </summary>
        private void RunMotorPIDLoops()
        {
            foreach (var config in _actuatorConfigurations.OfType<DriveMotorConfig>())
            {
                if (!config.isEnabled || !_driveMotorPIDs.ContainsKey(config.id)) continue;

                // Placeholder: Get actual current velocity from a sensor or motor component
                float currentVelocity = _currentMotorVelocities.ContainsKey(config.id) ? _currentMotorVelocities[config.id] : 0f;
                float targetVelocity = 0f; // This would come from a command queue or locomotion system

                // For boilerplate, let's assume a dummy target velocity for demonstration
                // In a real system, this would be set by a `SetMotorTargetVelocity` method.
                if (config.id == 0) targetVelocity = 10f; // Left motor dummy target
                if (config.id == 1) targetVelocity = 10f; // Right motor dummy target

                float error = targetVelocity - currentVelocity;
                float controlOutput = _driveMotorPIDs[config.id].Calculate(error, Time.fixedDeltaTime);

                // Clamp control output to max force/torque
                controlOutput = Mathf.Clamp(controlOutput, -config.maxForceTorque, config.maxForceTorque);

                // Placeholder: Apply this control output to the actual motor component
                // e.g., _driveMotorObjects[config.id].GetComponent<Rigidbody>().AddTorque(transform.up * controlOutput);
                // Debug.Log($"Motor {config.id}: Target={targetVelocity:F2}, Current={currentVelocity:F2}, Output={controlOutput:F2}");
            }
        }

        /// <summary>
        /// Runs PID control loops for all configured arm joints.
        /// This would read current joint angle feedback and apply torques.
        /// </summary>
        private void RunArmJointPIDLoops()
        {
            foreach (var config in _actuatorConfigurations.OfType<ArmJointConfig>())
            {
                if (!config.isEnabled || !_armJointPIDs.ContainsKey(config.id)) continue;

                // Placeholder: Get actual current angle from a sensor or joint component
                float currentAngle = _currentJointAngles.ContainsKey(config.id) ? _currentJointAngles[config.id] : 0f;
                float targetAngle = 0f; // This would come from a command queue or manipulation system

                // For boilerplate, let's assume a dummy target angle
                targetAngle = 45f; // Dummy target angle for all arm joints

                float error = targetAngle - currentAngle;
                float controlOutput = _armJointPIDs[config.id].Calculate(error, Time.fixedDeltaTime);

                // Clamp control output to max force/torque
                controlOutput = Mathf.Clamp(controlOutput, -config.maxForceTorque, config.maxForceTorque);

                // Placeholder: Apply this control output to the actual joint component
                // e.g., _armJointObjects[config.id].GetComponent<HingeJoint>().motor.force = controlOutput;
                // Debug.Log($"Joint {config.id}: Target={targetAngle:F2}, Current={currentAngle:F2}, Output={controlOutput:F2}");
            }
        }

        /// <summary>
        /// Commands a specific drive motor to a target velocity.
        /// </summary>
        /// <param name="motorID">The ID of the motor.</param>
        /// <param name="targetVelocity">Desired velocity (e.g., rad/s).</param>
        /// <param name="targetTorque">Desired torque (optional, for torque control).</param>
        public void CommandDriveMotor(int motorID, float targetVelocity, float targetTorque = 0f)
        {
            DriveMotorConfig config = _actuatorConfigurations.OfType<DriveMotorConfig>().FirstOrDefault(c => c.id == motorID);
            if (config == null || !config.isEnabled)
            {
                Debug.LogWarning($"ActuatorManager: Drive motor ID {motorID} not found or disabled.");
                OnActuatorError?.Invoke($"Drive motor ID {motorID} not found or disabled.");
                return;
            }

            // Here, you would update the target for the PID controller, not directly apply force.
            // _driveMotorPIDs[motorID].SetTarget(targetVelocity); // If PID had a SetTarget method
            Debug.Log($"ActuatorManager: Commanding motor {motorID} to velocity {targetVelocity:F2}, torque {targetTorque:F2}.");
            OnMotorCommandSent?.Invoke(new MotorCommand(motorID, targetVelocity, targetTorque, false));
        }

        /// <summary>
        /// Commands a specific arm joint to a target angle.
        /// </summary>
        /// <param name="jointID">The ID of the arm joint.</param>
        /// <param name="targetAngle">Desired angle (degrees).</param>
        public void CommandArmJoint(int jointID, float targetAngle)
        {
            ArmJointConfig config = _actuatorConfigurations.OfType<ArmJointConfig>().FirstOrDefault(c => c.id == jointID);
            if (config == null || !config.isEnabled)
            {
                Debug.LogWarning($"ActuatorManager: Arm joint ID {jointID} not found or disabled.");
                OnActuatorError?.Invoke($"Arm joint ID {jointID} not found or disabled.");
                return;
            }

            // Update the target for the joint's PID controller
            // _armJointPIDs[jointID].SetTarget(targetAngle);
            Debug.Log($"ActuatorManager: Commanding arm joint {jointID} to angle {targetAngle:F2} degrees.");
            OnJointCommandSent?.Invoke(jointID, targetAngle);
        }

        /// <summary>
        /// Commands the gripper to open or close.
        /// </summary>
        /// <param name="open">True to open, false to close.</param>
        public void CommandGripper(bool open)
        {
            GripperConfig config = _actuatorConfigurations.OfType<GripperConfig>().FirstOrDefault(); // Assuming one gripper
            if (config == null || !config.isEnabled)
            {
                Debug.LogWarning("ActuatorManager: Gripper not found or disabled.");
                OnActuatorError?.Invoke("Gripper not found or disabled.");
                return;
            }

            // Placeholder: Send command to gripper mechanism
            Debug.Log($"ActuatorManager: Commanding gripper to {(open ? "open" : "close")}.");
            OnGripperStateChanged?.Invoke(open);
        }

        /// <summary>
        /// Stops all actuators immediately.
        /// </summary>
        public void StopAllActuators()
        {
            Debug.Log("ActuatorManager: Stopping all actuators.");
            // Reset all PID controllers
            foreach (var pid in _driveMotorPIDs.Values) pid.Reset();
            foreach (var pid in _armJointPIDs.Values) pid.Reset();

            // Send zero commands to all active actuators
            foreach (var config in _actuatorConfigurations)
            {
                if (config.isEnabled)
                {
                    switch (config.type)
                    {
                        case ActuatorType.DriveMotor:
                            CommandDriveMotor(config.id, 0f, 0f);
                            break;
                        case ActuatorType.ArmJoint:
                            // Command to hold current position or move to a safe default
                            CommandArmJoint(config.id, _currentJointAngles.ContainsKey(config.id) ? _currentJointAngles[config.id] : 0f);
                            break;
                        case ActuatorType.Gripper:
                            // Gripper might stay in current state or move to a default.
                            // For simplicity, let's assume it stops moving.
                            break;
                    }
                }
            }
        }

        /// <summary>
        /// Sets the enabled state of a specific actuator by its ID.
        /// </summary>
        /// <param name="actuatorID">The ID of the actuator.</param>
        /// <param name="enable">True to enable, false to disable.</param>
        public void SetActuatorEnabled(int actuatorID, bool enable)
        {
            ActuatorConfig config = _actuatorConfigurations.FirstOrDefault(ac => ac.id == actuatorID);
            if (config != null)
            {
                config.isEnabled = enable;
                Debug.Log($"Actuator '{config.name}' (ID:{actuatorID}) {(enable ? "enabled" : "disabled")}.");
                // Reinitialize if enabling/disabling requires re-setup
                // InitializeActuators();
            }
            else
            {
                Debug.LogWarning($"ActuatorManager: Actuator ID {actuatorID} not found.");
            }
        }

        /// <summary>
        /// Retrieves an actuator's configuration by its ID.
        /// </summary>
        /// <param name="actuatorID">The ID of the actuator.</param>
        /// <returns>The ActuatorConfig, or null if not found.</returns>
        public ActuatorConfig GetActuatorConfig(int actuatorID)
        {
            return _actuatorConfigurations.FirstOrDefault(ac => ac.id == actuatorID);
        }

        /// <summary>
        /// Updates the internally tracked current velocity of a motor (e.g., from feedback).
        /// </summary>
        /// <param name="motorID">The ID of the motor.</param>
        /// <param name="velocity">The current velocity of the motor.</param>
        public void UpdateMotorFeedback(int motorID, float velocity)
        {
            _currentMotorVelocities[motorID] = velocity;
        }

        /// <summary>
        /// Updates the internally tracked current angle of an arm joint (e.g., from feedback).
        /// </summary>
        /// <param name="jointID">The ID of the joint.</param>
        /// <param name="angle">The current angle of the joint.</param>
        public void UpdateJointFeedback(int jointID, float angle)
        {
            _currentJointAngles[jointID] = angle;
        }
    }
}
