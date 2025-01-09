using UnityEngine;

public class PlayerController : MonoBehaviour
{
  [Header("References")]
  [SerializeField] private Camera playerCamera;
  [Header("Movement Settings")]
  [SerializeField] private float moveForce = 100f;
  [SerializeField] private float maxSpeed = 2000f;
  [SerializeField] private float groundDrag = 1f;
  public float initialForce = 5f;

  private Rigidbody rb;
  private bool isGrounded;
  private bool isControlledByAgent;

  private void Start()
  {
    rb = GetComponent<Rigidbody>();
    if (rb == null)
    {
      rb = gameObject.AddComponent<Rigidbody>();
      rb.maxAngularVelocity = maxSpeed;
      rb.constraints = RigidbodyConstraints.FreezeRotation;
    }
    rb.maxAngularVelocity = maxSpeed;

    // Check if there's an enabled ML-Agent component
    var agent = GetComponent<BoundarySequenceAgent>();
    isControlledByAgent = agent != null && agent.enabled;
  }

  // Public method to toggle control
  public void SetAgentControl(bool enabled)
  {
    isControlledByAgent = enabled;
  }

  public void Move(Vector2 movementInput)
  {
    if (!isGrounded) return;

    Vector3 localMovement = new Vector3(movementInput.x, 0f, movementInput.y);
    Vector3 movement = playerCamera != null ?
        playerCamera.transform.TransformDirection(localMovement) :
        localMovement;
    movement.y = 0;
    movement.Normalize();

    rb.AddForce(movement * moveForce, ForceMode.Force);

    Vector3 flatVel = new Vector3(rb.velocity.x, 0f, rb.velocity.z);
    if (flatVel.magnitude > maxSpeed)
    {
      Vector3 limitedVel = flatVel.normalized * maxSpeed;
      rb.velocity = new Vector3(limitedVel.x, rb.velocity.y, limitedVel.z);
    }
  }

  private void FixedUpdate()
  {
    CheckGrounded();
    ApplyDrag();

    // Only handle direct input when not controlled by ML-Agent
    if (!isControlledByAgent)
    {
      Vector2 input = new Vector2(
          Input.GetAxis("Horizontal"),
          Input.GetAxis("Vertical")
      );
      Move(input);
    }
  }

  private void CheckGrounded()
  {
    isGrounded = Physics.Raycast(transform.position, Vector3.down, 1.1f);
  }

  private void ApplyDrag()
  {
    rb.drag = isGrounded ? groundDrag : 0f;
  }

  public bool IsGrounded() => isGrounded;
  public Rigidbody GetRigidbody() => rb;
  public float GetMaxSpeed() => maxSpeed;
}