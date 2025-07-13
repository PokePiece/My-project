// File: Assets/Scripts/Boilerplate/RobotEnums.cs
// This file defines various enumerations used throughout the robot control system.
// It helps in categorizing different states, types, and modes, making the code
// more readable and maintainable.

using System;

namespace RobotBoilerplate
{
    // Enum for the overall operational mode of the robot.
    public enum RobotMode
    {
        /// <summary>
        /// The robot is idle and not performing any active tasks.
        /// It might be waiting for commands or in a low-power state.
        /// </summary>
        Idle,

        /// <summary>
        /// The robot is being controlled directly by a human operator (e.g., via joystick, keyboard).
        /// </summary>
        Teleoperated,

        /// <summary>
        /// The robot is executing a pre-programmed sequence of actions or navigating autonomously.
        /// </summary>
        Autonomous,

        /// <summary>
        /// The robot is in a safety-critical state, often triggered by an emergency stop or fault.
        /// All motion is typically halted immediately.
        /// </summary>
        EmergencyStop,

        /// <summary>
        /// The robot is performing a diagnostic routine or calibration.
        /// </summary>
        Diagnostic,

        /// <summary>
        /// The robot is currently charging its power source.
        /// </summary>
        Charging
    }

    // Enum for different types of sensors the robot might possess.
    public enum SensorType
    {
        /// <summary>
        /// Light Detection and Ranging sensor (for distance and mapping).
        /// </summary>
        Lidar,

        /// <summary>
        /// Standard camera for visual input.
        /// </summary>
        Camera,

        /// <summary>
        /// Inertial Measurement Unit (for orientation, angular velocity, and linear acceleration).
        /// </summary>
        IMU,

        /// <summary>
        /// Wheel encoders (for odometry and wheel speed).
        /// </summary>
        Encoder,

        /// <summary>
        /// Proximity sensors (e.g., ultrasonic, infrared) for short-range obstacle detection.
        /// </summary>
        Proximity,

        /// <summary>
        /// Force/Torque sensors (e.g., at end-effectors for manipulation).
        /// </summary>
        ForceTorque,

        /// <summary>
        /// Global Positioning System (for absolute position).
        /// </summary>
        GPS
    }

    // Enum for different types of actuators the robot might have.
    public enum ActuatorType
    {
        /// <summary>
        /// Drive motors for locomotion.
        /// </summary>
        DriveMotor,

        /// <summary>
        /// Gripper mechanism for grasping objects.
        /// </summary>
        Gripper,

        /// <summary>
        /// Robotic arm joints.
        /// </summary>
        ArmJoint,

        /// <summary>
        /// Panning/tilting camera mounts.
        /// </summary>
        PanTiltUnit,

        /// <summary>
        /// Linear actuators (e.g., for lifting mechanisms).
        /// </summary>
        LinearActuator
    }

    // Enum for different types of locomotion behaviors.
    public enum LocomotionMode
    {
        /// <summary>
        /// Direct velocity commands (linear and angular).
        /// </summary>
        VelocityControl,

        /// <summary>
        /// Moving to a specific target pose.
        /// </summary>
        PoseControl,

        /// <summary>
        /// Following a predefined path or trajectory.
        /// </summary>
        PathFollowing,

        /// <summary>
        /// Moving towards a goal while avoiding obstacles.
        /// </summary>
        Navigation,

        /// <summary>
        /// Stopping all locomotion.
        /// </summary>
        Stop
    }

    // Enum for different states of a task execution.
    public enum TaskState
    {
        /// <summary>
        /// The task is pending or not yet started.
        /// </summary>
        Pending,

        /// <summary>
        /// The task is currently being executed.
        /// </summary>
        InProgress,

        /// <summary>
        /// The task has completed successfully.
        /// </summary>
        Completed,

        /// <summary>
        /// The task failed to complete.
        /// </summary>
        Failed,

        /// <summary>
        /// The task was interrupted or cancelled.
        /// </summary>
        Cancelled
    }
}
