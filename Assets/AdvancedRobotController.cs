// File: Assets/Scripts/Boilerplate/AdvancedRobotController.cs
// This script serves as a comprehensive boilerplate for an advanced robot control system
// within Unity. It demonstrates a modular structure, common robot functionalities,
// and integration points for various subsystems.
//
// This is a boilerplate, not a fully implemented solution. Most methods contain
// placeholder logic and are intended to be expanded upon. Its purpose is to
// provide a large C# code sample and a robust architectural starting point.

using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq; // For LINQ operations

namespace RobotBoilerplate
{
    /// <summary>
    /// The main controller for the robot, managing its state, sensors, actuators,
    /// locomotion, and high-level behaviors.
    /// </summary>
    public class AdvancedRobotController : MonoBehaviour
    {
        [Header("Robot Configuration")]
        [Tooltip("The current operational mode of the robot.")]
        [SerializeField] private RobotMode _currentRobotMode = RobotMode.Idle;
        public RobotMode CurrentRobotMode => _currentRobotMode;

        [Tooltip("Configuration settings for the robot's physical properties and capabilities.")]
        [SerializeField] private RobotConfiguration _robotConfig = new RobotConfiguration();
        public RobotConfiguration RobotConfig => _robotConfig;

        [Tooltip("Reference to the robot's Rigidbody component for physics interactions.")]
        [SerializeField] private Rigidbody _robotRigidbody;

        [Tooltip("Transform representing the robot's base frame.")]
        [SerializeField] private Transform _robotBaseTransform;

        [Header("Locomotion Parameters")]
        [Tooltip("Current locomotion mode being executed.")]
        [SerializeField] private LocomotionMode _currentLocomotionMode = LocomotionMode.Stop;

        [Tooltip("Desired linear velocity for teleoperation/autonomous movement.")]
        [SerializeField] private float _desiredLinearVelocity = 0f;

        [Tooltip("Desired angular velocity for teleoperation/autonomous movement.")]
        [SerializeField] private float _desiredAngularVelocity = 0f;

        [Header("Sensor Readings (Debug)")]
        [Tooltip("Dictionary to store the latest sensor readings.")]
        private Dictionary<SensorType, SensorReading> _latestSensorReadings = new Dictionary<SensorType, SensorReading>();

        [Header("Actuator Commands (Debug)")]
        [Tooltip("List of current motor commands being sent to drive motors.")]
        private List<MotorCommand> _motorCommands = new List<MotorCommand>();

        [Header("State Estimation")]
        [Tooltip("The estimated current pose (position and orientation) of the robot.")]
        private Pose _currentEstimatedPose;
        public Pose CurrentEstimatedPose => _currentEstimatedPose;

        [Tooltip("The estimated current twist (linear and angular velocity) of the robot.")]
        private Twist _currentEstimatedTwist;
        public Twist CurrentEstimatedTwist => _currentEstimatedTwist;

        [Header("Task Management")]
        [Tooltip("The currently active robot task or mission.")]
        [SerializeField] private RobotTask _activeTask;
        public RobotTask ActiveTask => _activeTask;

        // Private fields for internal state management
        private Coroutine _currentLocomotionCoroutine;
        private bool _isInitialized = false;
        private float _lastSensorUpdateTime = 0f;
        private const float SensorUpdateInterval = 0.05f; // 20 Hz sensor update

        // --- Event System ---
        // Define C# events for various robot state changes, allowing other systems to subscribe.
        public event Action<RobotMode> OnRobotModeChanged;
        public event Action OnEmergencyStopTriggered;
        public event Action<LocomotionMode> OnLocomotionModeChanged;
        public event Action<Pose> OnPoseReached;
        public event Action<List<Waypoint>> OnPathCompleted;
        public event Action<SensorReading> OnSensorDataReceived;
        public event Action<List<MotorCommand>> OnMotorCommandsApplied;
        public event Action<bool> OnGripperCommanded;
        public event Action<int, float> OnArmJointCommanded;
        public event Action<Pose, Twist> OnRobotStateUpdated;
        public event Action<RobotTask> OnTaskStarted;
        public event Action<RobotTask> OnTaskProgressUpdated;
        public event Action<RobotTask, TaskState> OnTaskStateChanged;
        public event Action<string> OnErrorOccurred;
        public event Action<string> OnLogMessage; // For general logging

        // --- Unity Lifecycle Methods ---

        /// <summary>
        /// Called when the script instance is being loaded.
        /// Used for initialization that should happen before Start().
        /// </summary>
        private void Awake()
        {
            Debug.Log("AdvancedRobotController: Awake - Initializing robot systems.");

            // Ensure Rigidbody is assigned
            if (_robotRigidbody == null)
            {
                _robotRigidbody = GetComponent<Rigidbody>();
                if (_robotRigidbody == null)
                {
                    Debug.LogError("AdvancedRobotController: Rigidbody not found! Please assign one or ensure it's on the same GameObject.");
                    enabled = false; // Disable script if essential component is missing
                    return;
                }
            }

            // Ensure Base Transform is assigned
            if (_robotBaseTransform == null)
            {
                _robotBaseTransform = transform; // Default to this GameObject's transform
                Debug.LogWarning("AdvancedRobotController: Robot Base Transform not explicitly assigned. Using this GameObject's transform.");
            }

            // Initialize sensor readings dictionary
            foreach (SensorType type in Enum.GetValues(typeof(SensorType)))
            {
                _latestSensorReadings[type] = new SensorReading(type, type.ToString(), 0);
            }

            // Load default configuration
            LoadRobotConfiguration("DefaultConfig");

            _isInitialized = true;
            Debug.Log("AdvancedRobotController: Initialization complete.");
        }

        /// <summary>
        /// Called on the frame when a script is enabled just before any of the Update methods are called the first time.
        /// </summary>
        private void Start()
        {
            if (!_isInitialized) return;

            Debug.Log("AdvancedRobotController: Start - Robot ready to receive commands.");
            SetRobotMode(RobotMode.Idle);

            // Example: Start a coroutine for periodic sensor updates
            StartCoroutine(UpdateSensorsPeriodically());
        }

        /// <summary>
        /// Update is called once per frame.
        /// Used for game logic, input handling, and non-physics updates.
        /// </summary>
        private void Update()
        {
            if (!_isInitialized) return;

            // Example: Basic input for teleoperation
            if (_currentRobotMode == RobotMode.Teleoperated)
            {
                HandleTeleoperationInput();
            }

            // Update estimated pose and twist from physics (or more advanced methods)
            UpdateRobotStateFromPhysics();

            // Debug visualization (e.g., drawing paths, sensor rays)
            DrawDebugVisualizations();
        }

        /// <summary>
        /// FixedUpdate is called at a fixed framerate, independent of frame rate.
        /// Used for physics calculations and applying forces.
        /// </summary>
        private void FixedUpdate()
        {
            if (!_isInitialized) return;

            // Apply locomotion commands based on current mode
            ApplyLocomotionCommands();

            // Update actuator states based on commands
            UpdateActuators();
        }

        /// <summary>
        /// Called when the GameObject becomes enabled and active.
        /// </summary>
        private void OnEnable()
        {
            Debug.Log("AdvancedRobotController: OnEnable");
            // Potentially re-enable communication, resume tasks, etc.
        }

        /// <summary>
        /// Called when the GameObject becomes disabled or inactive.
        /// </summary>
        private void OnDisable()
        {
            Debug.Log("AdvancedRobotController: OnDisable");
            // Stop all motion, pause tasks, clean up resources.
            StopAllMotion();
            SetRobotMode(RobotMode.Idle);
            if (_currentLocomotionCoroutine != null)
            {
                StopCoroutine(_currentLocomotionCoroutine);
                _currentLocomotionCoroutine = null;
            }
        }

        /// <summary>
        /// Called when the MonoBehaviour will be destroyed.
        /// </summary>
        private void OnDestroy()
        {
            Debug.Log("AdvancedRobotController: OnDestroy - Cleaning up.");
            // Final cleanup of any unmanaged resources or external connections.
        }

        // --- Core Robot Control Methods ---

        /// <summary>
        /// Sets the overall operational mode of the robot.
        /// </summary>
        /// <param name="newMode">The new robot mode to set.</param>
        public void SetRobotMode(RobotMode newMode)
        {
            if (_currentRobotMode == newMode) return;

            Debug.Log($"Robot mode changing from {_currentRobotMode} to {newMode}");
            _currentRobotMode = newMode;

            // Handle mode transitions
            switch (newMode)
            {
                case RobotMode.Idle:
                    StopAllMotion();
                    CancelActiveTask();
                    break;
                case RobotMode.Teleoperated:
                    CancelActiveTask();
                    // Prepare for teleoperation (e.g., enable joystick input listeners)
                    break;
                case RobotMode.Autonomous:
                    // Prepare for autonomous operation (e.g., enable path planning, SLAM)
                    break;
                case RobotMode.EmergencyStop:
                    ForceEmergencyStop();
                    break;
                case RobotMode.Diagnostic:
                    // Enter diagnostic routines
                    break;
                case RobotMode.Charging:
                    StopAllMotion();
                    // Initiate charging sequence
                    break;
            }
            OnRobotModeChanged?.Invoke(newMode);
            OnLogMessage?.Invoke($"Robot mode set to: {newMode}");
        }

        /// <summary>
        /// Forces an immediate emergency stop, halting all robot motion and operations.
        /// </summary>
        public void ForceEmergencyStop()
        {
            Debug.LogWarning("AdvancedRobotController: EMERGENCY STOP ACTIVATED!");
            StopAllMotion();
            CancelActiveTask();
            _currentRobotMode = RobotMode.EmergencyStop; // Ensure mode is set
            OnEmergencyStopTriggered?.Invoke();
            OnErrorOccurred?.Invoke("Emergency Stop Triggered!");
        }

        /// <summary>
        /// Resets the robot from an emergency stop state to Idle.
        /// </summary>
        public void ResetEmergencyStop()
        {
            if (_currentRobotMode == RobotMode.EmergencyStop)
            {
                Debug.Log("AdvancedRobotController: Resetting from Emergency Stop.");
                SetRobotMode(RobotMode.Idle);
                OnLogMessage?.Invoke("Emergency Stop Reset.");
            }
        }

        // --- Locomotion Control ---

        /// <summary>
        /// Handles input for teleoperation mode.
        /// This is a placeholder for actual input reading (e.g., joystick, keyboard).
        /// </summary>
        private void HandleTeleoperationInput()
        {
            // Example: Read keyboard input for linear and angular velocity
            _desiredLinearVelocity = Input.GetAxis("Vertical") * _robotConfig.maxLinearSpeed;
            _desiredAngularVelocity = Input.GetAxis("Horizontal") * _robotConfig.maxAngularSpeed;

            SetLocomotionMode(LocomotionMode.VelocityControl);
        }

        /// <summary>
        /// Sets the current locomotion mode and potentially starts a new locomotion routine.
        /// </summary>
        /// <param name="mode">The desired locomotion mode.</param>
        /// <param name="targetPose">Optional: Target pose for pose control/navigation.</param>
        /// <param name="path">Optional: Path for path following.</param>
        public void SetLocomotionMode(LocomotionMode mode, Pose? targetPose = null, List<Waypoint> path = null)
        {
            if (_currentLocomotionMode == mode && mode != LocomotionMode.VelocityControl) return; // Allow continuous velocity updates

            if (_currentLocomotionCoroutine != null)
            {
                StopCoroutine(_currentLocomotionCoroutine);
                _currentLocomotionCoroutine = null;
            }

            _currentLocomotionMode = mode;
            Debug.Log($"Setting locomotion mode to: {mode}");

            switch (mode)
            {
                case LocomotionMode.VelocityControl:
                    // Velocity control is handled directly in FixedUpdate based on _desiredLinearVelocity/_desiredAngularVelocity
                    break;
                case LocomotionMode.PoseControl:
                    if (targetPose.HasValue)
                    {
                        _currentLocomotionCoroutine = StartCoroutine(MoveToPose(targetPose.Value));
                    }
                    else
                    {
                        Debug.LogError("PoseControl mode requires a target pose.");
                        OnErrorOccurred?.Invoke("PoseControl mode requires a target pose.");
                        SetLocomotionMode(LocomotionMode.Stop);
                    }
                    break;
                case LocomotionMode.PathFollowing:
                    if (path != null && path.Any())
                    {
                        _currentLocomotionCoroutine = StartCoroutine(FollowPath(path));
                    }
                    else
                    {
                        Debug.LogError("PathFollowing mode requires a path.");
                        OnErrorOccurred?.Invoke("PathFollowing mode requires a path.");
                        SetLocomotionMode(LocomotionMode.Stop);
                    }
                    break;
                case LocomotionMode.Navigation:
                    // This would typically involve a separate navigation stack (e.g., SLAM, path planning)
                    Debug.Log("Navigation mode initiated. (Requires external navigation system)");
                    OnLogMessage?.Invoke("Navigation mode initiated. (Requires external navigation system)");
                    // Start a navigation routine coroutine here
                    _currentLocomotionCoroutine = StartCoroutine(NavigateToGoal()); // Placeholder for navigation coroutine
                    break;
                case LocomotionMode.Stop:
                    StopAllMotion();
                    break;
            }
            OnLocomotionModeChanged?.Invoke(mode);
            OnLogMessage?.Invoke($"Locomotion mode set to: {mode}");
        }

        /// <summary>
        /// Stops all robot motion by setting velocities to zero.
        /// </summary>
        public void StopAllMotion()
        {
            _desiredLinearVelocity = 0f;
            _desiredAngularVelocity = 0f;
            if (_robotRigidbody != null)
            {
                _robotRigidbody.linearVelocity = Vector3.zero;
                _robotRigidbody.angularVelocity = Vector3.zero;
            }
            ApplyMotorCommands(new List<MotorCommand>()); // Send zero commands to motors
            Debug.Log("All robot motion stopped.");
            OnLogMessage?.Invoke("All robot motion stopped.");
        }

        /// <summary>
        /// Applies the current locomotion commands to the robot's Rigidbody.
        /// This is a simplified differential drive model.
        /// </summary>
        private void ApplyLocomotionCommands()
        {
            if (_currentRobotMode == RobotMode.EmergencyStop || _robotRigidbody == null) return;

            // Simple differential drive model for applying force
            // This assumes a flat ground and direct control over linear/angular velocity
            Vector3 targetVelocity = _robotBaseTransform.forward * _desiredLinearVelocity;
            Vector3 currentVelocity = _robotRigidbody.linearVelocity;

            // Calculate force needed to reach target linear velocity
            Vector3 linearForce = (targetVelocity - currentVelocity) * _robotRigidbody.mass * 10f; // P-controller like
            _robotRigidbody.AddForce(linearForce, ForceMode.Acceleration);

            // Calculate torque needed to reach target angular velocity
            float currentAngularVelocityY = _robotRigidbody.angularVelocity.y;
            float angularDifference = _desiredAngularVelocity - currentAngularVelocityY;
            float angularForce = angularDifference * _robotRigidbody.mass * 5f; // P-controller like
            _robotRigidbody.AddTorque(0, angularForce, 0, ForceMode.Acceleration);

            // Placeholder for individual motor control
            // In a real robot, _desiredLinearVelocity and _desiredAngularVelocity
            // would be translated into individual wheel velocities.
            TranslateVelocitiesToMotorCommands(_desiredLinearVelocity, _desiredAngularVelocity);
        }

        /// <summary>
        /// Translates desired linear and angular velocities into individual motor commands.
        /// </summary>
        /// <param name="linear">Desired linear velocity.</param>
        /// <param name="angular">Desired angular velocity.</param>
        private void TranslateVelocitiesToMotorCommands(float linear, float angular)
        {
            _motorCommands.Clear();
            float leftWheelVel = linear - (angular * _robotConfig.wheelBase / 2f);
            float rightWheelVel = linear + (angular * _robotConfig.wheelBase / 2f);

            // Convert to motor RPM/rad/s based on wheel radius
            // Assuming targetVelocity in MotorCommand is rad/s
            float leftMotorTarget = leftWheelVel / _robotConfig.wheelRadius;
            float rightMotorTarget = rightWheelVel / _robotConfig.wheelRadius;

            _motorCommands.Add(new MotorCommand(0, leftMotorTarget, 0, false)); // Left motor
            _motorCommands.Add(new MotorCommand(1, rightMotorTarget, 0, false)); // Right motor

            ApplyMotorCommands(_motorCommands);
        }

        /// <summary>
        /// Coroutine to move the robot to a specific target pose.
        /// </summary>
        /// <param name="targetPose">The target pose to reach.</param>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator MoveToPose(Pose targetPose)
        {
            Debug.Log($"Moving to target pose: {targetPose}");
            OnLogMessage?.Invoke($"Moving to target pose: {targetPose.position}");
            float positionTolerance = 0.1f; // meters
            float orientationTolerance = 5f; // degrees

            while (Vector3.Distance(_robotBaseTransform.position, targetPose.position) > positionTolerance ||
                   Quaternion.Angle(_robotBaseTransform.rotation, targetPose.orientation) > orientationTolerance)
            {
                if (_currentRobotMode == RobotMode.EmergencyStop)
                {
                    Debug.LogWarning("MoveToPose: Emergency Stop detected. Aborting.");
                    OnErrorOccurred?.Invoke("MoveToPose aborted due to Emergency Stop.");
                    yield break; // Exit if emergency stop
                }

                // Simple P-controller for movement
                Vector3 directionToTarget = (targetPose.position - _robotBaseTransform.position);
                float distance = directionToTarget.magnitude;
                directionToTarget.Normalize();

                _desiredLinearVelocity = distance > positionTolerance ? _robotConfig.maxLinearSpeed : 0;
                _desiredLinearVelocity = Mathf.Min(_desiredLinearVelocity, distance * 2f); // Decelerate as approaching

                // Simple P-controller for rotation
                Quaternion rotationDifference = Quaternion.Inverse(_robotBaseTransform.rotation) * targetPose.orientation;
                float angle = 0;
                Vector3 axis = Vector3.zero;
                rotationDifference.ToAngleAxis(out angle, out axis);

                // Ensure angle is between -180 and 180 for shortest path
                if (angle > 180f) angle -= 360f;
                if (angle < -180f) angle += 360f;

                float angularError = Vector3.Dot(axis, Vector3.up) * angle; // Only consider rotation around Y-axis
                _desiredAngularVelocity = angularError * 0.05f; // Proportional gain

                yield return new WaitForFixedUpdate(); // Wait for physics update
            }

            Debug.Log("Reached target pose.");
            OnLogMessage?.Invoke("Reached target pose.");
            StopAllMotion();
            SetLocomotionMode(LocomotionMode.Stop);
            OnPoseReached?.Invoke(targetPose);
        }

        /// <summary>
        /// Coroutine to follow a predefined path of waypoints.
        /// </summary>
        /// <param name="path">The list of waypoints to follow.</param>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator FollowPath(List<Waypoint> path)
        {
            Debug.Log($"Following path with {path.Count} waypoints.");
            OnLogMessage?.Invoke($"Starting path following with {path.Count} waypoints.");
            for (int i = 0; i < path.Count; i++)
            {
                Waypoint currentWaypoint = path[i];
                Debug.Log($"Navigating to waypoint {i + 1}: {currentWaypoint}");
                OnLogMessage?.Invoke($"Navigating to waypoint {i + 1}: {currentWaypoint.pose.position}");

                // Move to the waypoint's pose
                yield return StartCoroutine(MoveToPose(currentWaypoint.pose));

                if (_currentRobotMode == RobotMode.EmergencyStop)
                {
                    Debug.LogWarning("FollowPath: Emergency Stop detected. Aborting.");
                    OnErrorOccurred?.Invoke("Path following aborted due to Emergency Stop.");
                    yield break;
                }

                // If the waypoint requires a specific speed, apply it
                _desiredLinearVelocity = currentWaypoint.speed; // This would be part of a more advanced path controller

                // Placeholder for more complex path following logic (e.g., pure pursuit, PID control)
                yield return new WaitForSeconds(0.5f); // Small pause at waypoint for demonstration
            }
            Debug.Log("Path following completed.");
            OnLogMessage?.Invoke("Path following completed.");
            StopAllMotion();
            SetLocomotionMode(LocomotionMode.Stop);
            OnPathCompleted?.Invoke(path);
        }

        /// <summary>
        /// Coroutine for autonomous navigation to a goal.
        /// This would involve integration with a path planning and obstacle avoidance system.
        /// </summary>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator NavigateToGoal()
        {
            Debug.Log("Starting autonomous navigation routine.");
            OnLogMessage?.Invoke("Autonomous navigation initiated.");

            // Placeholder for a navigation loop:
            // 1. Get current pose
            // 2. Query path planner for next path segment/waypoint
            // 3. Execute movement to next waypoint (e.g., using MoveToPose or a dedicated local planner)
            // 4. Continuously check for obstacles and re-plan if necessary
            // 5. Monitor goal criteria

            bool goalReached = false;
            while (!goalReached)
            {
                if (_currentRobotMode != RobotMode.Autonomous)
                {
                    Debug.Log("Navigation routine interrupted due to mode change.");
                    OnErrorOccurred?.Invoke("Navigation interrupted: Robot mode changed.");
                    yield break;
                }
                if (_currentRobotMode == RobotMode.EmergencyStop)
                {
                    Debug.LogWarning("NavigateToGoal: Emergency Stop detected. Aborting.");
                    OnErrorOccurred?.Invoke("Navigation aborted due to Emergency Stop.");
                    yield break;
                }

                // --- Placeholder: Path Planning & Obstacle Avoidance ---
                // In a real system, this would involve:
                // - Calling a global path planner to get a high-level path
                // - Calling a local path planner/obstacle avoidance system to generate
                //   safe velocities or micro-movements based on local sensor data.

                // For now, let's simulate some progress or a simple movement
                Debug.Log("Navigation: Simulating movement towards goal...");
                // Example: Move forward slowly
                _desiredLinearVelocity = 0.5f;
                _desiredAngularVelocity = 0f;

                // Check for simulated obstacles or goal proximity
                if (Vector3.Distance(_robotBaseTransform.position, Vector3.forward * 10f) < 1f) // Dummy goal at (0,0,10)
                {
                    goalReached = true;
                    Debug.Log("Simulated goal reached!");
                    OnLogMessage?.Invoke("Simulated navigation goal reached.");
                    StopAllMotion();
                }

                yield return new WaitForFixedUpdate(); // Or a more intelligent wait based on planning cycle
            }

            Debug.Log("Autonomous navigation completed.");
            SetLocomotionMode(LocomotionMode.Stop);
            OnLogMessage?.Invoke("Autonomous navigation completed.");
        }


        // --- Sensor Processing ---

        /// <summary>
        /// Coroutine to simulate periodic sensor updates.
        /// In a real system, this would be driven by actual sensor data streams.
        /// </summary>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator UpdateSensorsPeriodically()
        {
            while (true)
            {
                if (_currentRobotMode == RobotMode.EmergencyStop)
                {
                    yield return new WaitForSeconds(1.0f); // Wait longer if in emergency stop
                    continue;
                }

                long timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

                // Simulate Lidar data (e.g., detecting obstacles in front)
                if (_robotConfig.enabledSensors.ContainsKey(SensorType.Lidar) && _robotConfig.enabledSensors[SensorType.Lidar])
                {
                    RaycastHit hit;
                    if (Physics.Raycast(_robotBaseTransform.position, _robotBaseTransform.forward, out hit, 5f))
                    {
                        SensorReading lidarReading = new SensorReading(SensorType.Lidar, "FrontLidar", timestamp);
                        lidarReading.valueFloat = hit.distance;
                        lidarReading.valueVector = hit.point;
                        _latestSensorReadings[SensorType.Lidar] = lidarReading;
                        OnSensorDataReceived?.Invoke(lidarReading);
                        Debug.DrawRay(_robotBaseTransform.position, _robotBaseTransform.forward * hit.distance, Color.red, SensorUpdateInterval);
                    }
                    else
                    {
                        SensorReading lidarReading = new SensorReading(SensorType.Lidar, "FrontLidar", timestamp);
                        lidarReading.valueFloat = float.PositiveInfinity;
                        _latestSensorReadings[SensorType.Lidar] = lidarReading;
                        OnSensorDataReceived?.Invoke(lidarReading);
                        Debug.DrawRay(_robotBaseTransform.position, _robotBaseTransform.forward * 5f, Color.green, SensorUpdateInterval);
                    }
                }

                // Simulate IMU data (angular velocity, linear acceleration)
                if (_robotConfig.enabledSensors.ContainsKey(SensorType.IMU) && _robotConfig.enabledSensors[SensorType.IMU])
                {
                    SensorReading imuReading = new SensorReading(SensorType.IMU, "MainIMU", timestamp);
                    imuReading.valueVector = _robotRigidbody.angularVelocity; // Angular velocity
                    imuReading.valueVector = _robotRigidbody.linearVelocity; // Linear velocity (simplified)
                    imuReading.valueQuaternion = _robotBaseTransform.rotation; // Orientation
                    _latestSensorReadings[SensorType.IMU] = imuReading;
                    OnSensorDataReceived?.Invoke(imuReading);
                }

                // Simulate Encoder data (wheel speeds)
                if (_robotConfig.enabledSensors.ContainsKey(SensorType.Encoder) && _robotConfig.enabledSensors[SensorType.Encoder])
                {
                    SensorReading encoderReading = new SensorReading(SensorType.Encoder, "WheelEncoders", timestamp);
                    // In a real robot, this would be actual encoder counts/velocities.
                    // Here, we'll use the current linear velocity as a proxy.
                    encoderReading.valueFloat = _robotRigidbody.linearVelocity.magnitude;
                    _latestSensorReadings[SensorType.Encoder] = encoderReading;
                    OnSensorDataReceived?.Invoke(encoderReading);
                }

                // Simulate Proximity sensor data
                if (_robotConfig.enabledSensors.ContainsKey(SensorType.Proximity) && _robotConfig.enabledSensors[SensorType.Proximity])
                {
                    RaycastHit proxHit;
                    if (Physics.Raycast(_robotBaseTransform.position + _robotBaseTransform.forward * 0.5f, _robotBaseTransform.forward, out proxHit, 0.5f))
                    {
                        SensorReading proxReading = new SensorReading(SensorType.Proximity, "FrontProximity", timestamp);
                        proxReading.valueFloat = proxHit.distance;
                        _latestSensorReadings[SensorType.Proximity] = proxReading;
                        OnSensorDataReceived?.Invoke(proxReading);
                        Debug.DrawRay(_robotBaseTransform.position + _robotBaseTransform.forward * 0.5f, _robotBaseTransform.forward * proxHit.distance, Color.yellow, SensorUpdateInterval);
                    }
                }

                // Placeholder for other sensor types (Camera, GPS, ForceTorque)
                // These would involve more complex data structures and processing.
                // For Camera: Capture a RenderTexture or process image data.
                // For GPS: Read simulated GPS coordinates.
                // For ForceTorque: Read forces/torques at specific points.

                yield return new WaitForSeconds(SensorUpdateInterval);
            }
        }

        /// <summary>
        /// Retrieves the latest reading from a specific sensor type.
        /// </summary>
        /// <param name="type">The type of sensor to retrieve data for.</param>
        /// <returns>The latest SensorReading, or a default empty one if not found.</returns>
        public SensorReading GetLatestSensorReading(SensorType type)
        {
            if (_latestSensorReadings.ContainsKey(type))
            {
                return _latestSensorReadings[type];
            }
            Debug.LogWarning($"No sensor data found for type: {type}");
            OnErrorOccurred?.Invoke($"Attempted to get data for unsupported sensor type: {type}");
            return new SensorReading(type, "N/A", 0);
        }

        // --- Actuator Control ---

        /// <summary>
        /// Applies a list of motor commands to the robot's drive motors.
        /// In a real robot, this would send commands to motor controllers.
        /// </summary>
        /// <param name="commands">List of motor commands.</param>
        private void ApplyMotorCommands(List<MotorCommand> commands)
        {
            _motorCommands = commands; // Store for debugging/inspection
            // Placeholder: In a real system, iterate through commands and send to hardware interface
            foreach (var cmd in commands)
            {
                // Debug.Log($"Applying motor command: {cmd}");
                // Example: Send command to a specific motor component/script
                // MotorController motor = GetMotorController(cmd.motorID);
                // if (motor != null) motor.SetVelocity(cmd.targetVelocity);
            }
            OnMotorCommandsApplied?.Invoke(commands);
        }

        /// <summary>
        /// Updates the state of all actuators based on their current commands.
        /// </summary>
        private void UpdateActuators()
        {
            // Placeholder: This might involve PID loops for individual motors,
            // or sending commands to a robotic arm controller.
            // Example: Check if motors are stalled, update gripper state, etc.
        }

        /// <summary>
        /// Commands the robot's gripper to open or close.
        /// </summary>
        /// <param name="open">True to open, false to close.</param>
        public void CommandGripper(bool open)
        {
            Debug.Log($"Commanding gripper to {(open ? "open" : "close")}");
            OnLogMessage?.Invoke($"Gripper commanded to {(open ? "open" : "close")}");
            // Placeholder: Send command to gripper actuator
            OnGripperCommanded?.Invoke(open);
        }

        /// <summary>
        /// Commands a specific joint of a robotic arm to a target angle.
        /// </summary>
        /// <param name="jointID">The ID of the arm joint.</param>
        /// <param name="targetAngle">The target angle in degrees.</param>
        public void CommandArmJoint(int jointID, float targetAngle)
        {
            Debug.Log($"Commanding arm joint {jointID} to {targetAngle} degrees.");
            OnLogMessage?.Invoke($"Arm joint {jointID} commanded to {targetAngle} degrees.");
            // Placeholder: Send command to arm joint actuator
            OnArmJointCommanded?.Invoke(jointID, targetAngle);
        }

        // --- State Estimation and Localization ---

        /// <summary>
        /// Updates the robot's estimated pose and twist based on Unity's physics.
        /// In a real robot, this would involve odometry, IMU integration, SLAM, etc.
        /// </summary>
        private void UpdateRobotStateFromPhysics()
        {
            long timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

            _currentEstimatedPose = new Pose(
                _robotBaseTransform.position,
                _robotBaseTransform.rotation,
                timestamp
            );

            _currentEstimatedTwist = new Twist(
                _robotRigidbody.linearVelocity,
                _robotRigidbody.angularVelocity,
                timestamp
            );

            OnRobotStateUpdated?.Invoke(_currentEstimatedPose, _currentEstimatedTwist);
        }

        /// <summary>
        /// Placeholder for a more advanced localization algorithm (e.g., SLAM, AMCL).
        /// This method would typically integrate sensor data (Lidar, IMU, GPS)
        /// to provide a more accurate and robust pose estimate than simple odometry.
        /// </summary>
        public void PerformLocalization()
        {
            Debug.Log("Performing advanced localization...");
            OnLogMessage?.Invoke("Initiating localization process.");
            // This would involve:
            // 1. Collecting sensor data (e.g., Lidar scans, IMU data, GPS fixes).
            // 2. Processing data through a localization algorithm (e.g., Extended Kalman Filter, Particle Filter).
            // 3. Updating _currentEstimatedPose and _currentEstimatedTwist with the refined estimate.
            // 4. Potentially publishing the localized pose to other systems.

            // Simulate a localization update
            // For a real system, this would be a complex, continuous process.
            _currentEstimatedPose.position += new Vector3(UnityEngine.Random.Range(-0.01f, 0.01f), 0, UnityEngine.Random.Range(-0.01f, 0.01f));
            _currentEstimatedPose.orientation = Quaternion.Euler(0, UnityEngine.Random.Range(-0.5f, 0.5f), 0) * _currentEstimatedPose.orientation;
            _currentEstimatedPose.timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

            Debug.Log($"Localization updated. New estimated pose: {_currentEstimatedPose}");
            OnLogMessage?.Invoke($"Localization updated. New estimated pose: {_currentEstimatedPose.position}");
            OnRobotStateUpdated?.Invoke(_currentEstimatedPose, _currentEstimatedTwist); // Re-invoke with refined pose
        }

        // --- Task Management ---

        /// <summary>
        /// Assigns and starts a new robot task.
        /// </summary>
        /// <param name="task">The RobotTask to execute.</param>
        public void StartTask(RobotTask task)
        {
            if (_activeTask != null && _activeTask.currentState == TaskState.InProgress)
            {
                Debug.LogWarning($"Cannot start new task '{task.taskName}'. Task '{_activeTask.taskName}' is already in progress.");
                OnErrorOccurred?.Invoke($"Failed to start task: Task '{_activeTask.taskName}' already in progress.");
                return;
            }

            _activeTask = task;
            _activeTask.SetState(TaskState.InProgress);
            Debug.Log($"Starting task: {_activeTask.taskName} (ID: {_activeTask.taskID})");
            OnLogMessage?.Invoke($"Starting task: {_activeTask.taskName}");
            OnTaskStarted?.Invoke(_activeTask);

            // Based on task type, initiate appropriate locomotion/action
            switch (_activeTask.actionType)
            {
                case "Navigate":
                    if (_activeTask.path != null && _activeTask.path.Any())
                    {
                        SetLocomotionMode(LocomotionMode.PathFollowing, path: _activeTask.path);
                    }
                    else if (_activeTask.targetObjectID != string.Empty)
                    {
                        // Assume targetObjectID implies navigation to that object's position
                        // Placeholder: Find object's pose and navigate
                        Debug.Log($"Attempting to navigate to target object: {_activeTask.targetObjectID}");
                        OnLogMessage?.Invoke($"Attempting to navigate to target object: {_activeTask.targetObjectID}");
                        // Example: find a GameObject with the targetObjectID and get its pose
                        // GameObject target = GameObject.Find(_activeTask.targetObjectID);
                        // if (target != null)
                        // {
                        //     SetLocomotionMode(LocomotionMode.PoseControl, new Pose(target.transform.position, target.transform.rotation, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()));
                        // }
                        // else
                        // {
                        //     Debug.LogError($"Target object '{_activeTask.targetObjectID}' not found for navigation.");
                        //     _activeTask.SetState(TaskState.Failed, "Target object not found.");
                        //     OnErrorOccurred?.Invoke($"Task failed: Target object '{_activeTask.targetObjectID}' not found.");
                        // }
                    }
                    else
                    {
                        Debug.LogError($"Navigation task '{_activeTask.taskName}' has no path or target object.");
                        _activeTask.SetState(TaskState.Failed, "No navigation target specified.");
                        OnErrorOccurred?.Invoke($"Task failed: No navigation target for task '{_activeTask.taskName}'.");
                    }
                    break;
                case "Manipulate":
                    // Placeholder: Start manipulation sequence (e.g., pick and place)
                    Debug.Log($"Initiating manipulation task: {_activeTask.taskName}");
                    OnLogMessage?.Invoke($"Initiating manipulation task: {_activeTask.taskName}");
                    StartCoroutine(ExecuteManipulationTask(_activeTask));
                    break;
                case "Inspect":
                    // Placeholder: Start inspection routine (e.g., move camera, take picture)
                    Debug.Log($"Initiating inspection task: {_activeTask.taskName}");
                    OnLogMessage?.Invoke($"Initiating inspection task: {_activeTask.taskName}");
                    StartCoroutine(ExecuteInspectionTask(_activeTask));
                    break;
                default:
                    Debug.LogError($"Unsupported task action type: {_activeTask.actionType}");
                    _activeTask.SetState(TaskState.Failed, $"Unsupported action type: {_activeTask.actionType}");
                    OnErrorOccurred?.Invoke($"Task failed: Unsupported action type '{_activeTask.actionType}'.");
                    break;
            }
            SetRobotMode(RobotMode.Autonomous); // Switch to autonomous mode for task execution
        }

        /// <summary>
        /// Cancels the currently active task.
        /// </summary>
        public void CancelActiveTask()
        {
            if (_activeTask != null && _activeTask.currentState == TaskState.InProgress)
            {
                Debug.Log($"Cancelling active task: {_activeTask.taskName}");
                OnLogMessage?.Invoke($"Cancelling active task: {_activeTask.taskName}");
                _activeTask.SetState(TaskState.Cancelled, "Task cancelled by user/system.");
                OnTaskStateChanged?.Invoke(_activeTask, TaskState.Cancelled);
                StopAllMotion(); // Stop any ongoing locomotion related to the task
            }
            _activeTask = null; // Clear active task
        }

        /// <summary>
        /// Coroutine to execute a manipulation task.
        /// </summary>
        /// <param name="task">The manipulation task details.</param>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator ExecuteManipulationTask(RobotTask task)
        {
            Debug.Log($"Executing manipulation task: {task.taskName}");
            OnLogMessage?.Invoke($"Executing manipulation task: {task.taskName}");
            task.UpdateProgress(0.1f);
            OnTaskProgressUpdated?.Invoke(task);

            // Placeholder: Move arm to pre-grasp pose
            CommandArmJoint(0, 90f); // Example joint command
            yield return new WaitForSeconds(1.0f);

            // Placeholder: Move to target object
            // yield return StartCoroutine(MoveToPose(GetPoseOfObject(task.targetObjectID)));

            // Placeholder: Open gripper
            CommandGripper(true);
            yield return new WaitForSeconds(0.5f);

            // Placeholder: Grasp object
            CommandGripper(false);
            yield return new WaitForSeconds(1.0f);

            // Placeholder: Move arm to post-grasp pose / drop-off pose
            CommandArmJoint(0, 0f);
            yield return new WaitForSeconds(1.0f);

            task.UpdateProgress(1.0f);
            task.SetState(TaskState.Completed);
            Debug.Log($"Manipulation task '{task.taskName}' completed.");
            OnLogMessage?.Invoke($"Manipulation task '{task.taskName}' completed.");
            OnTaskStateChanged?.Invoke(task, TaskState.Completed);
            _activeTask = null; // Task finished
            SetRobotMode(RobotMode.Idle); // Return to idle
        }

        /// <summary>
        /// Coroutine to execute an inspection task.
        /// </summary>
        /// <param name="task">The inspection task details.</param>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator ExecuteInspectionTask(RobotTask task)
        {
            Debug.Log($"Executing inspection task: {task.taskName}");
            OnLogMessage?.Invoke($"Executing inspection task: {task.taskName}");
            task.UpdateProgress(0.2f);
            OnTaskProgressUpdated?.Invoke(task);

            // Placeholder: Navigate to inspection point
            // yield return StartCoroutine(MoveToPose(GetInspectionPose(task.targetObjectID)));

            // Placeholder: Activate camera, take picture/scan
            Debug.Log("Taking inspection image...");
            OnLogMessage?.Invoke("Taking inspection image...");
            yield return new WaitForSeconds(2.0f); // Simulate camera capture

            // Placeholder: Process image or send to external system
            Debug.Log("Image processed. Inspection complete.");
            OnLogMessage?.Invoke("Inspection complete.");

            task.UpdateProgress(1.0f);
            task.SetState(TaskState.Completed);
            Debug.Log($"Inspection task '{task.taskName}' completed.");
            OnLogMessage?.Invoke($"Inspection task '{task.taskName}' completed.");
            OnTaskStateChanged?.Invoke(task, TaskState.Completed);
            _activeTask = null; // Task finished
            SetRobotMode(RobotMode.Idle); // Return to idle
        }

        // --- Configuration Management ---

        /// <summary>
        /// Loads robot configuration from a specified source (e.g., file, database).
        /// </summary>
        /// <param name="configName">The name of the configuration to load.</param>
        public void LoadRobotConfiguration(string configName)
        {
            Debug.Log($"Loading robot configuration: {configName}");
            OnLogMessage?.Invoke($"Loading configuration: {configName}");
            // Placeholder: In a real application, this would load from disk,
            // a remote server, or a configuration manager.
            // For now, it just initializes with default values or dummy data.
            _robotConfig = new RobotConfiguration { configName = configName };
            _robotConfig.enabledSensors[SensorType.Lidar] = true;
            _robotConfig.enabledSensors[SensorType.IMU] = true;
            _robotConfig.enabledSensors[SensorType.Encoder] = true;
            _robotConfig.enabledSensors[SensorType.Proximity] = true;
            _robotConfig.activeAlgorithms.Add("BasicObstacleAvoidance");
            _robotConfig.activeAlgorithms.Add("SimplePathFollowing");

            Debug.Log($"Configuration '{configName}' loaded successfully.");
            OnLogMessage?.Invoke($"Configuration '{configName}' loaded.");
        }

        /// <summary>
        /// Saves the current robot configuration to a persistent storage.
        /// </summary>
        /// <param name="configName">The name to save the configuration as.</param>
        public void SaveRobotConfiguration(string configName)
        {
            Debug.Log($"Saving robot configuration: {configName}");
            OnLogMessage?.Invoke($"Saving configuration: {configName}");
            _robotConfig.configName = configName;
            string json = _robotConfig.SaveToJson(); // Uses the dummy SaveToJson from RobotConfiguration
            Debug.Log($"Configuration saved: {json}");
            OnLogMessage?.Invoke($"Configuration '{configName}' saved.");
        }

        /// <summary>
        /// Applies a new set of configuration settings to the robot.
        /// This might involve re-initializing subsystems.
        /// </summary>
        /// <param name="newConfig">The new RobotConfiguration object.</param>
        public void ApplyConfiguration(RobotConfiguration newConfig)
        {
            Debug.Log($"Applying new configuration: {newConfig.configName}");
            OnLogMessage?.Invoke($"Applying new configuration: {newConfig.configName}");
            _robotConfig = newConfig;
            // Placeholder: Re-initialize sensor managers, locomotion parameters, etc.
            // based on the new configuration.
            Debug.Log("New configuration applied.");
        }

        // --- Communication Interface (Placeholder) ---

        /// <summary>
        /// Establishes a connection to the physical robot hardware or a communication middleware.
        /// </summary>
        /// <param name="connectionString">Details for connection (e.g., IP address, port).</param>
        public void ConnectToHardware(string connectionString)
        {
            Debug.Log($"Attempting to connect to hardware: {connectionString}");
            OnLogMessage?.Invoke($"Attempting to connect to hardware: {connectionString}");
            // Placeholder: Initialize TCP/IP, UDP, Serial, or ROS connection.
            // If successful, set isConnected = true;
            bool isConnected = true; // Simulate connection success
            if (isConnected)
            {
                Debug.Log("Hardware connected successfully.");
                OnLogMessage?.Invoke("Hardware connected successfully.");
                // Start listening for incoming data (sensor readings, feedback)
                // Start sending outgoing commands (motor commands, actuator commands)
            }
            else
            {
                Debug.LogError("Failed to connect to hardware.");
                OnErrorOccurred?.Invoke("Failed to connect to hardware.");
            }
        }

        /// <summary>
        /// Disconnects from the physical robot hardware.
        /// </summary>
        public void DisconnectFromHardware()
        {
            Debug.Log("Disconnecting from hardware.");
            OnLogMessage?.Invoke("Disconnecting from hardware.");
            // Placeholder: Close all communication channels.
            // If successful, set isConnected = false;
            Debug.Log("Hardware disconnected.");
            OnLogMessage?.Invoke("Hardware disconnected.");
        }

        /// <summary>
        /// Sends a generic command message to the robot hardware.
        /// </summary>
        /// <param name="message">The command message string.</param>
        public void SendHardwareCommand(string message)
        {
            Debug.Log($"Sending hardware command: {message}");
            // Placeholder: Serialize message and send over established connection.
            // This could be JSON, Protobuf, or a custom binary protocol.
        }

        /// <summary>
        /// Receives a generic data message from the robot hardware.
        /// This would typically be called by an asynchronous communication listener.
        /// </summary>
        /// <param name="data">The received data string.</param>
        private void ReceiveHardwareData(string data)
        {
            Debug.Log($"Received hardware data: {data}");
            // Placeholder: Deserialize data and update sensor readings, robot state, etc.
            // Example: if data is "LIDAR: 1.5m", parse it and update _latestSensorReadings.
        }

        // --- Debugging and Visualization ---

        /// <summary>
        /// Draws various debug visualizations in the Unity editor.
        /// </summary>
        private void DrawDebugVisualizations()
        {
            // Draw robot's current velocity vector
            Debug.DrawRay(_robotBaseTransform.position, _robotRigidbody.linearVelocity, Color.blue);

            // Draw robot's forward direction
            Debug.DrawRay(_robotBaseTransform.position, _robotBaseTransform.forward * 1f, Color.cyan);

            // Draw target linear velocity (if teleoperated or autonomous)
            if (_currentRobotMode == RobotMode.Teleoperated || _currentRobotMode == RobotMode.Autonomous)
            {
                Debug.DrawRay(_robotBaseTransform.position + Vector3.up * 0.1f, _robotBaseTransform.forward * _desiredLinearVelocity, Color.magenta);
            }

            // Draw path waypoints if an active task has a path
            if (_activeTask != null && _activeTask.path != null && _activeTask.path.Any())
            {
                for (int i = 0; i < _activeTask.path.Count; i++)
                {
                    Color waypointColor = (i == 0) ? Color.yellow : Color.gray; // Highlight first waypoint
                    Debug.DrawLine(_activeTask.path[i].pose.position - Vector3.up * 0.5f, _activeTask.path[i].pose.position + Vector3.up * 0.5f, waypointColor);
                    if (i > 0)
                    {
                        Debug.DrawLine(_activeTask.path[i-1].pose.position, _activeTask.path[i].pose.position, Color.white);
                    }
                }
            }
        }

        /// <summary>
        /// Logs a message to the Unity console and potentially to a UI log.
        /// </summary>
        /// <param name="message">The message to log.</param>
        public void Log(string message)
        {
            Debug.Log($"[RobotLog] {message}");
            OnLogMessage?.Invoke(message);
        }

        /// <summary>
        /// Logs an error message.
        /// </summary>
        /// <param name="errorMessage">The error message.</param>
        public void LogError(string errorMessage)
        {
            Debug.LogError($"[RobotError] {errorMessage}");
            OnErrorOccurred?.Invoke(errorMessage);
        }

        // --- Utility Methods ---

        /// <summary>
        /// Calculates the current battery level (placeholder).
        /// </summary>
        /// <returns>Battery level between 0 and 1.</returns>
        public float GetBatteryLevel()
        {
            // Placeholder: In a real robot, read from power management system.
            return 0.75f; // Simulate 75% battery
        }

        /// <summary>
        /// Checks if the robot is currently connected to its physical hardware.
        /// </summary>
        /// <returns>True if connected, false otherwise.</returns>
        public bool IsHardwareConnected()
        {
            // Placeholder: Check communication status
            return true; // Simulate always connected for now
        }

        /// <summary>
        /// Retrieves a reference to a specific motor controller component (placeholder).
        /// </summary>
        /// <param name="motorID">The ID of the motor.</param>
        /// <returns>A reference to the MotorController script, or null if not found.</returns>
        // private MotorController GetMotorController(int motorID)
        // {
        //     // Placeholder: Implement a way to find and return specific motor scripts
        //     // e.g., from a list of child GameObjects or a dictionary.
        //     return null;
        // }

        /// <summary>
        /// Retrieves the pose of a target object in the scene (placeholder).
        /// </summary>
        /// <param name="objectID">The ID of the object.</param>
        /// <returns>The pose of the object, or Pose.Identity if not found.</returns>
        private Pose GetPoseOfObject(string objectID)
        {
            // Placeholder: Find GameObject by name/tag/ID and return its pose.
            GameObject obj = GameObject.Find(objectID);
            if (obj != null)
            {
                return new Pose(obj.transform.position, obj.transform.rotation, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());
            }
            Debug.LogWarning($"Object with ID '{objectID}' not found in scene.");
            return Pose.Identity;
        }

        /// <summary>
        /// Generates an inspection pose for a given object (placeholder).
        /// </summary>
        /// <param name="objectID">The ID of the object to inspect.</param>
        /// <returns>A suitable inspection pose.</returns>
        private Pose GetInspectionPose(string objectID)
        {
            Pose objectPose = GetPoseOfObject(objectID);
            // Placeholder: Calculate a pose slightly offset from the object for inspection.
            return new Pose(objectPose.position + objectPose.orientation * new Vector3(0, 0.5f, -1.0f), objectPose.orientation, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());
        }

        // --- End of AdvancedRobotController ---
    }
}
