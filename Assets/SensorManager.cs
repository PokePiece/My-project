// File: Assets/Scripts/Boilerplate/SensorManager.cs
// This script manages all sensors attached to the robot, handling data acquisition,
// processing, and providing filtered or raw sensor readings to other robot systems.
// It is designed to be a central point for sensor configuration and data access.

using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq; // For LINQ operations

namespace RobotBoilerplate
{
    /// <summary>
    /// Configuration settings for a specific sensor.
    /// </summary>
    [Serializable]
    public class SensorConfig
    {
        public SensorType type;                 // The type of sensor.
        public string name;                     // Unique name for this sensor instance (e.g., "FrontLidar", "LeftCamera").
        public bool isEnabled;                  // Whether this sensor is active.
        public float updateFrequencyHz;         // How often to update this sensor's data.
        public float noiseStdDev;               // Standard deviation of simulated noise for this sensor.
        public Vector3 mountOffset;             // Local position offset from the robot's base.
        public Quaternion mountRotation;        // Local rotation offset from the robot's base.
        public float maxRange;                  // Max detection range for distance sensors.
        public float fieldOfView;               // Field of view for cameras/lidars.

        public SensorConfig(SensorType type, string name, bool enabled = true, float freq = 10f, float noise = 0f)
        {
            this.type = type;
            this.name = name;
            this.isEnabled = enabled;
            this.updateFrequencyHz = freq;
            this.noiseStdDev = noise;
            this.mountOffset = Vector3.zero;
            this.mountRotation = Quaternion.identity;
            this.maxRange = 10f;
            this.fieldOfView = 60f;
        }

        public override string ToString()
        {
            return $"{name} ({type}) - Enabled: {isEnabled}, Freq: {updateFrequencyHz}Hz";
        }
    }

    /// <summary>
    /// Manages all sensor data acquisition, processing, and distribution.
    /// This script should be attached to the robot's root GameObject.
    /// </summary>
    public class SensorManager : MonoBehaviour
    {
        [Header("Sensor Configuration")]
        [Tooltip("List of all sensors configured for this robot.")]
        [SerializeField] private List<SensorConfig> _sensorConfigurations = new List<SensorConfig>();

        [Tooltip("Reference to the robot's base transform for sensor mounting.")]
        [SerializeField] private Transform _robotBaseTransform;

        [Header("Sensor Data Output (Debug)")]
        [Tooltip("Dictionary storing the latest processed sensor readings.")]
        private Dictionary<string, SensorReading> _latestSensorReadings = new Dictionary<string, SensorReading>();
        public IReadOnlyDictionary<string, SensorReading> LatestSensorReadings => _latestSensorReadings;

        // Private fields for internal management
        private Dictionary<string, Coroutine> _sensorUpdateCoroutines = new Dictionary<string, Coroutine>();
        private bool _isInitialized = false;

        // --- Event System ---
        public event Action<SensorReading> OnNewSensorData; // Event for new data from any sensor
        public event Action<SensorType, SensorReading> OnSpecificSensorData; // Event for specific sensor type data

        /// <summary>
        /// Called when the script instance is being loaded.
        /// </summary>
        private void Awake()
        {
            Debug.Log("SensorManager: Awake - Initializing.");

            if (_robotBaseTransform == null)
            {
                _robotBaseTransform = transform;
                Debug.LogWarning("SensorManager: Robot Base Transform not assigned. Using this GameObject's transform.");
            }

            InitializeSensors();
            _isInitialized = true;
        }

        /// <summary>
        /// Called on the frame when a script is enabled just before any of the Update methods are called the first time.
        /// </summary>
        private void Start()
        {
            if (!_isInitialized) return;
            Debug.Log("SensorManager: Start - Sensor data acquisition initiated.");
        }

        /// <summary>
        /// Called when the GameObject becomes disabled or inactive.
        /// </summary>
        private void OnDisable()
        {
            Debug.Log("SensorManager: OnDisable - Stopping all sensor update routines.");
            StopAllSensorUpdates();
        }

        /// <summary>
        /// Called when the MonoBehaviour will be destroyed.
        /// </summary>
        private void OnDestroy()
        {
            Debug.Log("SensorManager: OnDestroy - Cleaning up.");
            StopAllSensorUpdates();
        }

        /// <summary>
        /// Initializes all configured sensors and starts their respective update routines.
        /// </summary>
        private void InitializeSensors()
        {
            StopAllSensorUpdates(); // Ensure no old coroutines are running

            if (_sensorConfigurations == null || _sensorConfigurations.Count == 0)
            {
                Debug.LogWarning("SensorManager: No sensor configurations found. Adding default IMU sensor.");
                _sensorConfigurations.Add(new SensorConfig(SensorType.IMU, "DefaultIMU", true, 50f));
            }

            foreach (var config in _sensorConfigurations)
            {
                if (config.isEnabled)
                {
                    Debug.Log($"SensorManager: Starting update for {config.name} ({config.type}) at {config.updateFrequencyHz} Hz.");
                    Coroutine updateRoutine = StartCoroutine(SensorUpdateRoutine(config));
                    _sensorUpdateCoroutines[config.name] = updateRoutine;
                }
                else
                {
                    Debug.Log($"SensorManager: Sensor {config.name} ({config.type}) is disabled.");
                }
            }
        }

        /// <summary>
        /// Stops all active sensor update coroutines.
        /// </summary>
        private void StopAllSensorUpdates()
        {
            foreach (var entry in _sensorUpdateCoroutines)
            {
                if (entry.Value != null)
                {
                    StopCoroutine(entry.Value);
                }
            }
            _sensorUpdateCoroutines.Clear();
        }

        /// <summary>
        /// Coroutine for a single sensor's data acquisition and processing loop.
        /// </summary>
        /// <param name="config">The configuration for the sensor.</param>
        /// <returns>IEnumerator for the coroutine.</returns>
        private IEnumerator SensorUpdateRoutine(SensorConfig config)
        {
            float updateInterval = 1.0f / config.updateFrequencyHz;
            while (true)
            {
                long timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                SensorReading newReading = new SensorReading(config.type, config.name, timestamp);

                // Simulate data acquisition based on sensor type
                switch (config.type)
                {
                    case SensorType.Lidar:
                        newReading = SimulateLidarReading(config, timestamp);
                        break;
                    case SensorType.Camera:
                        newReading = SimulateCameraReading(config, timestamp);
                        break;
                    case SensorType.IMU:
                        newReading = SimulateImuReading(config, timestamp);
                        break;
                    case SensorType.Encoder:
                        newReading = SimulateEncoderReading(config, timestamp);
                        break;
                    case SensorType.Proximity:
                        newReading = SimulateProximityReading(config, timestamp);
                        break;
                    case SensorType.ForceTorque:
                        newReading = SimulateForceTorqueReading(config, timestamp);
                        break;
                    case SensorType.GPS:
                        newReading = SimulateGpsReading(config, timestamp);
                        break;
                    default:
                        Debug.LogWarning($"SensorManager: Unsupported sensor type for simulation: {config.type}");
                        break;
                }

                // Apply noise
                newReading = ApplyNoise(newReading, config.noiseStdDev);

                // Store and publish the reading
                _latestSensorReadings[config.name] = newReading;
                OnNewSensorData?.Invoke(newReading);
                OnSpecificSensorData?.Invoke(config.type, newReading);

                yield return new WaitForSeconds(updateInterval);
            }
        }

        /// <summary>
        /// Simulates a Lidar sensor reading.
        /// </summary>
        private SensorReading SimulateLidarReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            Vector3 sensorWorldPos = _robotBaseTransform.position + _robotBaseTransform.TransformDirection(config.mountOffset);
            Quaternion sensorWorldRot = _robotBaseTransform.rotation * config.mountRotation;
            Vector3 rayDirection = sensorWorldRot * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(sensorWorldPos, rayDirection, out hit, config.maxRange))
            {
                reading.valueFloat = hit.distance;
                reading.valueVector = hit.point;
                Debug.DrawRay(sensorWorldPos, rayDirection * hit.distance, Color.red, 1.0f / config.updateFrequencyHz);
            }
            else
            {
                reading.valueFloat = float.PositiveInfinity;
                Debug.DrawRay(sensorWorldPos, rayDirection * config.maxRange, Color.green, 1.0f / config.updateFrequencyHz);
            }
            return reading;
        }

        /// <summary>
        /// Simulates a Camera sensor reading.
        /// </summary>
        private SensorReading SimulateCameraReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            // Placeholder: In a real simulation, you'd render to a RenderTexture
            // and potentially process it. For boilerplate, we just set a dummy value.
            reading.rawData = "Simulated image data (placeholder)";
            return reading;
        }

        /// <summary>
        /// Simulates an IMU sensor reading (acceleration, angular velocity, orientation).
        /// Assumes an attached Rigidbody for real-time physics data.
        /// </summary>
        private SensorReading SimulateImuReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            Rigidbody rb = _robotBaseTransform.GetComponentInParent<Rigidbody>();
            if (rb != null)
            {
                // Linear acceleration (simplified: change in velocity over time)
                // More accurately: (rb.velocity - previousVelocity) / Time.fixedDeltaTime
                reading.valueVector = rb.velocity; // Using velocity as a placeholder for linear motion
                reading.valueVector = rb.angularVelocity; // Angular velocity
                reading.valueQuaternion = _robotBaseTransform.rotation; // Orientation
            }
            else
            {
                Debug.LogWarning("IMU simulation requires a Rigidbody on the robot.");
            }
            return reading;
        }

        /// <summary>
        /// Simulates Encoder readings (wheel speeds/positions).
        /// </summary>
        private SensorReading SimulateEncoderReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            // Placeholder: In a real robot, this would be actual encoder counts.
            // Here, we'll use the robot's current linear speed as a proxy for wheel speed.
            Rigidbody rb = _robotBaseTransform.GetComponentInParent<Rigidbody>();
            if (rb != null)
            {
                reading.valueFloat = rb.velocity.magnitude; // Linear speed as a proxy for wheel speed
                reading.rawData = $"LeftWheel: {rb.velocity.magnitude * 10f}, RightWheel: {rb.velocity.magnitude * 10f}";
            }
            return reading;
        }

        /// <summary>
        /// Simulates a Proximity sensor reading.
        /// </summary>
        private SensorReading SimulateProximityReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            Vector3 sensorWorldPos = _robotBaseTransform.position + _robotBaseTransform.TransformDirection(config.mountOffset);
            Quaternion sensorWorldRot = _robotBaseTransform.rotation * config.mountRotation;
            Vector3 rayDirection = sensorWorldRot * Vector3.forward;

            RaycastHit hit;
            if (Physics.Raycast(sensorWorldPos, rayDirection, out hit, config.maxRange))
            {
                reading.valueFloat = hit.distance;
            }
            else
            {
                reading.valueFloat = float.PositiveInfinity;
            }
            Debug.DrawRay(sensorWorldPos, rayDirection * reading.valueFloat, Color.yellow, 1.0f / config.updateFrequencyHz);
            return reading;
        }

        /// <summary>
        /// Simulates a Force/Torque sensor reading.
        /// </summary>
        private SensorReading SimulateForceTorqueReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            // Placeholder: Simulate some random force/torque
            reading.valueVector = new Vector3(UnityEngine.Random.Range(-1f, 1f), UnityEngine.Random.Range(-1f, 1f), UnityEngine.Random.Range(-1f, 1f)); // Force
            reading.valueVector = new Vector3(UnityEngine.Random.Range(-0.1f, 0.1f), UnityEngine.Random.Range(-0.1f, 0.1f), UnityEngine.Random.Range(-0.1f, 0.1f)); // Torque
            return reading;
        }

        /// <summary>
        /// Simulates a GPS sensor reading.
        /// </summary>
        private SensorReading SimulateGpsReading(SensorConfig config, long timestamp)
        {
            SensorReading reading = new SensorReading(config.type, config.name, timestamp);
            // Placeholder: Simulate GPS coordinates based on world position
            reading.valueVector = _robotBaseTransform.position; // Using Unity world position as lat/lon/alt proxy
            reading.rawData = $"Lat: {_robotBaseTransform.position.z}, Lon: {_robotBaseTransform.position.x}, Alt: {_robotBaseTransform.position.y}";
            return reading;
        }

        /// <summary>
        /// Applies simulated Gaussian noise to a sensor reading.
        /// </summary>
        /// <param name="reading">The original sensor reading.</param>
        /// <param name="stdDev">The standard deviation of the noise.</param>
        /// <returns>The noisy sensor reading.</returns>
        private SensorReading ApplyNoise(SensorReading reading, float stdDev)
        {
            if (stdDev <= 0) return reading;

            SensorReading noisyReading = reading; // Create a copy

            // Apply noise based on the data type in the reading
            if (reading.valueFloat != 0)
            {
                noisyReading.valueFloat += GetGaussianNoise(stdDev);
            }
            if (reading.valueVector != Vector3.zero)
            {
                noisyReading.valueVector.x += GetGaussianNoise(stdDev);
                noisyReading.valueVector.y += GetGaussianNoise(stdDev);
                noisyReading.valueVector.z += GetGaussianNoise(stdDev);
            }
            // Noise for Quaternion is more complex, often applied to Euler angles then converted back
            // For simplicity, we'll skip complex noise on Quaternion for this boilerplate.

            return noisyReading;
        }

        /// <summary>
        /// Generates a random number from a Gaussian (normal) distribution.
        /// </summary>
        /// <param name="stdDev">Standard deviation of the distribution.</param>
        /// <param name="mean">Mean of the distribution (defaults to 0).</param>
        /// <returns>A random float from a Gaussian distribution.</returns>
        private float GetGaussianNoise(float stdDev, float mean = 0f)
        {
            // Box-Muller transform to generate Gaussian distributed random numbers
            float u1 = 1.0f - UnityEngine.Random.value; // uniform(0,1] random number
            float u2 = 1.0f - UnityEngine.Random.value; // uniform(0,1] random number
            float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2); // random normal(0,1)
            return mean + stdDev * randStdNormal; // random normal(mean, stdDev^2)
        }

        /// <summary>
        /// Retrieves the latest sensor reading by its name.
        /// </summary>
        /// <param name="sensorName">The unique name of the sensor.</param>
        /// <returns>The latest SensorReading, or a default empty one if not found.</returns>
        public SensorReading GetSensorReadingByName(string sensorName)
        {
            if (_latestSensorReadings.TryGetValue(sensorName, out SensorReading reading))
            {
                return reading;
            }
            Debug.LogWarning($"SensorManager: No reading found for sensor named: {sensorName}");
            return new SensorReading(); // Return default empty struct
        }

        /// <summary>
        /// Retrieves the latest sensor reading by its type (returns the first one found).
        /// </summary>
        /// <param name="sensorType">The type of sensor.</param>
        /// <returns>The latest SensorReading, or a default empty one if not found.</returns>
        public SensorReading GetSensorReadingByType(SensorType sensorType)
        {
            var reading = _latestSensorReadings.Values.FirstOrDefault(sr => sr.type == sensorType);
            if (reading.sensorName != null) // Check if a valid reading was found
            {
                return reading;
            }
            Debug.LogWarning($"SensorManager: No reading found for sensor type: {sensorType}");
            return new SensorReading();
        }

        /// <summary>
        /// Sets the enabled state of a specific sensor by its name.
        /// </summary>
        /// <param name="sensorName">The name of the sensor.</param>
        /// <param name="enable">True to enable, false to disable.</param>
        public void SetSensorEnabled(string sensorName, bool enable)
        {
            SensorConfig config = _sensorConfigurations.FirstOrDefault(sc => sc.name == sensorName);
            if (config != null)
            {
                config.isEnabled = enable;
                Debug.Log($"Sensor '{sensorName}' {(enable ? "enabled" : "disabled")}. Reinitializing sensors.");
                InitializeSensors(); // Reinitialize to apply changes
            }
            else
            {
                Debug.LogWarning($"SensorManager: Sensor '{sensorName}' not found in configurations.");
            }
        }

        /// <summary>
        /// Applies a new set of sensor configurations, reinitializing all sensors.
        /// </summary>
        /// <param name="newConfigs">The new list of sensor configurations.</param>
        public void ApplySensorConfigurations(List<SensorConfig> newConfigs)
        {
            if (newConfigs == null)
            {
                Debug.LogError("SensorManager: Cannot apply null sensor configurations.");
                return;
            }
            _sensorConfigurations = newConfigs;
            Debug.Log("SensorManager: Applying new sensor configurations. Reinitializing sensors.");
            InitializeSensors();
        }
    }
}
