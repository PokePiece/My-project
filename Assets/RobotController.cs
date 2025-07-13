using UnityEngine;

// This script controls the movement of a simple robot.
public class RobotController : MonoBehaviour
{
    // Public variables allow us to adjust these values directly in the Unity Inspector.
    public float moveSpeed = 5f;    // Speed at which the robot moves forward/backward.
    public float turnSpeed = 100f;  // Speed at which the robot turns left/right.

    private Rigidbody rb; // Reference to the Rigidbody component of the robot body.

    // Start is called before the first frame update.
    // This is where we initialize components.
    void Start()
    {
        // Get the Rigidbody component attached to the same GameObject as this script.
        // The Rigidbody is essential for physics-based movement.
        rb = GetComponent<Rigidbody>();

        // Check if Rigidbody is found. If not, log an error.
        if (rb == null)
        {
            Debug.LogError("Rigidbody component not found on the RobotBody! Please add one.");
        }
    }

    // FixedUpdate is called at a fixed framerate, ideal for physics calculations.
    void FixedUpdate()
    {
        // Get input from the horizontal and vertical axes.
        // "Vertical" maps to W/S keys or Up/Down arrows.
        // "Horizontal" maps to A/D keys or Left/Right arrows.
        float verticalInput = Input.GetAxis("Vertical");
        float horizontalInput = Input.GetAxis("Horizontal");

        // --- Movement (Forward/Backward) ---
        // Calculate the movement direction based on the robot's forward vector
        // and the vertical input (W/S).
        // Time.fixedDeltaTime ensures movement is frame-rate independent.
        Vector3 moveDirection = transform.forward * verticalInput * moveSpeed * Time.fixedDeltaTime;

        // Apply the movement to the Rigidbody's position.
        // Using MovePosition is preferred for physics objects as it respects collisions.
        rb.MovePosition(rb.position + moveDirection);

        // --- Rotation (Left/Right) ---
        // Calculate the rotation amount based on the horizontal input (A/D).
        // Quaternion.Euler creates a rotation from Euler angles (degrees).
        // We only rotate around the Y-axis (up/down axis in Unity).
        Quaternion turnRotation = Quaternion.Euler(0f, horizontalInput * turnSpeed * Time.fixedDeltaTime, 0f);

        // Apply the rotation to the Rigidbody's rotation.
        // Using MoveRotation is preferred for physics objects.
        rb.MoveRotation(rb.rotation * turnRotation);
    }
}
