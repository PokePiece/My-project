// File: Assets/Scripts/Boilerplate/RobotDataStructures.cs
// This file defines various data structures (structs and classes) that represent
// information exchanged within the robot control system, such as sensor readings,
// actuator commands, and robot state.

using UnityEngine; // Required for Vector3, Quaternion, etc.
using System;
using System.Collections.Generic;

namespace RobotBoilerplate
{
    /// <summary>
    /// Represents a command to a single motor.
    /// </summary>
    [Serializable] // Makes it visible in Unity Inspector
    public struct MotorCommand
    {
        public int motorID;             // Unique identifier for the motor.
        public float targetVelocity;    // Desired velocity (e.g., rad/s or RPM).
        public float targetTorque;      // Desired torque (e.g., Nm).
        public bool enableBrake;        // Whether to engage the brake.

        public MotorCommand(int id, float velocity, float torque, bool brake)
        {
            motorID = id;
            targetVelocity = velocity;
            targetTorque = torque;
            enableBrake = brake;
        }

        public override string ToString()
        {
            return $"Motor {motorID}: Vel={targetVelocity:F2}, Torque={targetTorque:F2}, Brake={enableBrake}";
        }
    }

    /// <summary>
    /// Represents a generic sensor reading.
    /// </summary>
    [Serializable]
    public struct SensorReading
    {
        public SensorType type;         // Type of sensor.
        public string sensorName;       // Name of the specific sensor instance.
        public long timestamp;          // Timestamp of the reading (e.g., milliseconds since epoch).
        public Vector3 valueVector;     // Generic vector value (e.g., acceleration, angular velocity).
        public float valueFloat;        // Generic float value (e.g., distance, temperature).
        public Quaternion valueQuaternion; // Generic quaternion value (e.g., orientation).
        public string rawData;          // Raw string data for complex sensors (e.g., JSON for camera data).

        public SensorReading(SensorType type, string name, long timestamp)
        {
            this.type = type;
            this.sensorName = name;
            this.timestamp = timestamp;
            this.valueVector = Vector3.zero;
            this.valueFloat = 0f;
            this.valueQuaternion = Quaternion.identity;
            this.rawData = string.Empty;
        }

        public override string ToString()
        {
            return $"[{type}] {sensorName} @ {timestamp}: Vec={valueVector}, Float={valueFloat:F2}";
        }
    }

    /// <summary>
    /// Represents the 3D pose (position and orientation) of the robot.
    /// </summary>
    [Serializable]
    public struct Pose
    {
        public Vector3 position;        // Position in world coordinates.
        public Quaternion orientation;  // Orientation as a quaternion.
        public long timestamp;          // Timestamp of when this pose was valid.

        public Pose(Vector3 pos, Quaternion rot, long ts)
        {
            position = pos;
            orientation = rot;
            timestamp = ts;
        }

        public static Pose Identity => new Pose(Vector3.zero, Quaternion.identity, 0);

        public override string ToString()
        {
            return $"Pos: {position}, Rot: {orientation.eulerAngles}, TS: {timestamp}";
        }
    }

    /// <summary>
    /// Represents the 3D twist (linear and angular velocity) of the robot.
    /// </summary>
    [Serializable]
    public struct Twist
    {
        public Vector3 linear;          // Linear velocity (m/s).
        public Vector3 angular;         // Angular velocity (rad/s).
        public long timestamp;          // Timestamp of when this twist was valid.

        public Twist(Vector3 lin, Vector3 ang, long ts)
        {
            linear = lin;
            angular = ang;
            timestamp = ts;
        }

        public static Twist Zero => new Twist(Vector3.zero, Vector3.zero, 0);

        public override string ToString()
        {
            return $"Linear: {linear}, Angular: {angular}, TS: {timestamp}";
        }
    }

    /// <summary>
    /// Represents a single waypoint in a path or trajectory.
    /// </summary>
    [Serializable]
    public struct Waypoint
    {
        public Pose pose;               // Target pose for this waypoint.
        public float speed;             // Desired speed at this waypoint.
        public float tolerance;         // Positional tolerance for reaching this waypoint.
        public bool isKeyframe;         // True if this is a critical point in the path.

        public Waypoint(Pose p, float s, float t, bool kf = false)
        {
            pose = p;
            speed = s;
            tolerance = t;
            isKeyframe = kf;
        }

        public override string ToString()
        {
            return $"Waypoint: {pose}, Speed: {speed:F2}, Tol: {tolerance:F2}";
        }
    }

    /// <summary>
    /// Represents a configuration profile for the robot.
    /// </summary>
    [Serializable]
    public class RobotConfiguration
    {
        public string configName;                   // Name of this configuration.
        public float maxLinearSpeed;                // Maximum linear speed (m/s).
        public float maxAngularSpeed;               // Maximum angular speed (rad/s).
        public float wheelRadius;                   // Radius of the robot's wheels (m).
        public float wheelBase;                     // Distance between wheels (m).
        public Dictionary<SensorType, bool> enabledSensors; // Which sensors are active.
        public List<string> activeAlgorithms;       // List of active algorithms (e.g., "SLAM", "PathPlanner").

        public RobotConfiguration()
        {
            configName = "Default";
            maxLinearSpeed = 1.0f;
            maxAngularSpeed = 1.5f;
            wheelRadius = 0.1f;
            wheelBase = 0.3f;
            enabledSensors = new Dictionary<SensorType, bool>();
            activeAlgorithms = new List<string>();

            // Initialize default sensor states
            foreach (SensorType type in Enum.GetValues(typeof(SensorType)))
            {
                enabledSensors[type] = false;
            }
            enabledSensors[SensorType.IMU] = true; // IMU typically always on
        }

        public void LoadFromJson(string jsonString)
        {
            // Placeholder: In a real app, you'd deserialize JSON here.
            Debug.Log($"Loading configuration from JSON: {jsonString}");
        }

        public string SaveToJson()
        {
            // Placeholder: In a real app, you'd serialize to JSON here.
            Debug.Log("Saving configuration to JSON.");
            return JsonUtility.ToJson(this, true); // Basic Unity JSON serialization
        }

        public override string ToString()
        {
            return $"Config: {configName}, MaxLinVel: {maxLinearSpeed}, MaxAngVel: {maxAngularSpeed}";
        }
    }

    /// <summary>
    /// Represents a specific task or mission for the robot to execute.
    /// </summary>
    [Serializable]
    public class RobotTask
    {
        public string taskID;                   // Unique ID for the task.
        public string taskName;                 // Human-readable name.
        public TaskState currentState;          // Current state of the task.
        public List<Waypoint> path;             // Path to follow for navigation tasks.
        public string actionType;               // e.g., "Navigate", "Manipulate", "Inspect".
        public string targetObjectID;           // ID of an object to interact with.
        public float progress;                  // Task progress (0.0 to 1.0).
        public string errorMessage;             // Any error message if task failed.

        public RobotTask(string id, string name, string action)
        {
            taskID = id;
            taskName = name;
            actionType = action;
            currentState = TaskState.Pending;
            path = new List<Waypoint>();
            progress = 0f;
            errorMessage = string.Empty;
        }

        public void AddWaypoint(Waypoint wp)
        {
            path.Add(wp);
        }

        public void UpdateProgress(float newProgress)
        {
            progress = Mathf.Clamp01(newProgress);
        }

        public void SetState(TaskState newState, string error = "")
        {
            currentState = newState;
            errorMessage = error;
        }

        public override string ToString()
        {
            return $"Task '{taskName}' ({taskID}): {currentState}, Progress: {progress:P0}";
        }
    }
}
