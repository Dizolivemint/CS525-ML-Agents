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
  [SerializeField] private float sphereRadius = 0.5f; 

  private Rigidbody rb;
  private SphereCollider sphereCollider;
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
      rb.interpolation = RigidbodyInterpolation.Interpolate;
      rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
    }
    rb.maxAngularVelocity = maxSpeed;

    sphereCollider = GetComponent<SphereCollider>();
    if (sphereCollider == null)
    {
      sphereCollider = gameObject.AddComponent<SphereCollider>();
      sphereCollider.radius = sphereRadius;
      sphereCollider.material = CreateDefaultPhysicsMaterial();
    }

    // Check if there's an enabled ML-Agent component
    var agent = GetComponent<BoundarySequenceAgent>();
    isControlledByAgent = agent != null && agent.enabled;
  }

  private PhysicMaterial CreateDefaultPhysicsMaterial()
  {
    PhysicMaterial material = new PhysicMaterial("PlayerPhysicsMaterial");
    material.dynamicFriction = 0.6f;
    material.staticFriction = 0.6f;
    material.bounciness = 0.2f;
    material.frictionCombine = PhysicMaterialCombine.Average;
    material.bounceCombine = PhysicMaterialCombine.Average;
    return material;
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
    float radius = sphereCollider.radius;
    Vector3 origin = transform.position + Vector3.up * radius;
    float maxDistance = 1.1f + radius;
    isGrounded = Physics.SphereCast(origin, radius * 0.9f, Vector3.down, out _, maxDistance);
  }

  private void ApplyDrag()
  {
    rb.drag = isGrounded ? groundDrag : 0f;
  }

  public bool IsGrounded() => isGrounded;
  public Rigidbody GetRigidbody() => rb;
  public float GetMaxSpeed() => maxSpeed;
}