using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
  // Movement settings
  [Header("Movement")]
  public float rollSpeed = 200f;
  public float glideSpeed = 200f;
  public float initialForce = 1000f;
  public Vector3 initialDirection = Vector3.forward;

  [Header("ML-Agent Integration")]
  public float currentGravity;
  private bool isControlledByML = true;  // Flag to switch between ML and human control

  // Gravity Settings
  [Header("Gravity")]
  public float gliderGravity = 0.1f;
  public float minGravity = 20f;
  public float maxGravity = 10000f;   // Increased from 20
  public float gravityChangeSpeed = 1000f;  // Increased for faster changes

  // Rolling Settings
  [Header("Rolling Physics")]
  public float rollingDrag = 0.5f;      // Drastically reduced drag
  public float glidingDrag = 0.1f;       // Reduced gliding drag too
  public float slopeFactor = 2f;

  // Ground Check Settings
  [Header("Ground Detection")]
  public float groundCheckDistance = 1f;
  public float slopeDetectionRange = 2f;

  // Transform settings
  [Header("Transform")]
  public float sphereRadius = 0.5f;
  public float discRadius = 1f;
  public float discHeight = 0.1f;

  // Physics Thresholds
  [Header("Physics")]
  public float minimumVelocity = 20f;
  public float velocityDamping = 0.999f;

  [Header("Momentum")]
  public float momentumConversionFactor = 2f;  // How efficiently gravity converts to speed
  public float minimumForwardSpeed = 10f;

  [Header("Physics Safety")]
  public float maxVelocity = 200f;               // Cap on maximum velocity
  public int physicsIterations = 3;              // Number of collision iterations
  public float collisionBuffer = 0.05f;          // Minimum distance to maintain from surfaces
  public LayerMask terrainMask;                  // Layer mask for terrain checks
  private Vector3 lastValidPosition;             // Store last safe position

  [Header("Debug")]
  public bool showDebugGizmos = true;

  // References and state
  private Rigidbody rb;
  private bool isDisc = false;
  private Vector3 moveDirection;
  private MeshFilter meshFilter;
  private MeshCollider meshCollider;
  private bool hasInitialMomentum = false;
  private Quaternion targetRotation;
  private Vector3 lastVelocity;
  private float currentSlope;
  private bool isGoingDownhill;

  // Cached meshes for both forms
  private Mesh sphereMesh;
  private Mesh discMesh;

  void Start()
  {
    rb = GetComponent<Rigidbody>();
    meshFilter = GetComponent<MeshFilter>();
    meshCollider = GetComponent<MeshCollider>();

    if (rb == null || meshFilter == null || meshCollider == null)
    {
      Debug.LogError("Missing required components!");
      return;
    }

    // Set physics solver iterations
    rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
    Physics.defaultSolverIterations = physicsIterations;

    CreateMeshes();
    currentGravity = minGravity;
    targetRotation = transform.rotation;
    lastValidPosition = transform.position;
    StartCoroutine(ApplyInitialMomentum());
  }

  public void SetMLControl(bool enabled)
  {
    isControlledByML = enabled;
    if (!enabled)
    {
      currentGravity = minGravity;
    }
  }

  void Update()
  {
    if (!isControlledByML)
    {
      float vertical = Input.GetAxisRaw("Vertical");
      if (vertical > 0)
      {
        float gravityDelta = gravityChangeSpeed * Time.deltaTime;
        currentGravity = Mathf.Min(currentGravity + gravityDelta, maxGravity);
      }
      else
      {
        currentGravity = minGravity;
      }
    }
  }

  void FixedUpdate()
  {
    if (!hasInitialMomentum) return;

    // Store current position as potentially valid
    if (IsPositionSafe(transform.position))
    {
      lastValidPosition = transform.position;
    }

    // Apply gravity force
    rb.AddForce(Vector3.down * currentGravity, ForceMode.Force);

    HandleRolling();

    // Cap velocity
    if (rb.velocity.magnitude > maxVelocity)
    {
      rb.velocity = rb.velocity.normalized * maxVelocity;
    }

    // Check if we're in a valid position
    if (!IsPositionSafe(transform.position))
    {
      HandleTerrainPenetration();
    }
  }

  bool IsPositionSafe(Vector3 position)
  {
    // Check if position is inside terrain
    Collider[] overlaps = Physics.OverlapSphere(position, sphereRadius * 0.9f, terrainMask);
    return overlaps.Length == 0;
  }

  void HandleTerrainPenetration()
  {
    // First try: Cast in direction of velocity to find surface
    RaycastHit hit;
    if (Physics.Raycast(lastValidPosition, rb.velocity.normalized, out hit, rb.velocity.magnitude * Time.fixedDeltaTime, terrainMask))
    {
      // Position slightly above hit point
      Vector3 safePosition = hit.point + hit.normal * (sphereRadius + collisionBuffer);
      if (IsPositionSafe(safePosition))
      {
        transform.position = safePosition;
        // Deflect velocity along surface
        rb.velocity = Vector3.Reflect(rb.velocity, hit.normal) * 0.8f;
        return;
      }
    }

    // Fallback: Return to last valid position
    transform.position = lastValidPosition;
    // Maintain horizontal velocity but reduce vertical
    rb.velocity = new Vector3(rb.velocity.x, Mathf.Min(rb.velocity.y, 0), rb.velocity.z);
  }

  void HandleRolling()
  {
    if (moveDirection != Vector3.zero)
    {
      rb.AddForce(moveDirection * rollSpeed, ForceMode.Force);
    }

    // Convert gravity directly into forward momentum
    Vector3 currentForward = rb.velocity.normalized;
    if (currentForward != Vector3.zero)
    {
      float momentumForce = currentGravity * momentumConversionFactor;
      rb.AddForce(currentForward * momentumForce, ForceMode.Acceleration);
    }

    rb.AddForce(Vector3.down * currentGravity, ForceMode.Acceleration);

    // Ensure minimum speed with safety check
    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
    if (horizontalVelocity.magnitude < minimumForwardSpeed && IsPositionSafe(transform.position))
    {
      Vector3 newVelocity = horizontalVelocity.normalized * minimumForwardSpeed;
      newVelocity.y = rb.velocity.y;
      rb.velocity = newVelocity;
    }
  }
  void CreateMeshes()
  {
    GameObject tempSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
    sphereMesh = tempSphere.GetComponent<MeshFilter>().sharedMesh;
    Destroy(tempSphere);

    GameObject tempCylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
    discMesh = tempCylinder.GetComponent<MeshFilter>().sharedMesh;
    Destroy(tempCylinder);
  }

  IEnumerator ApplyInitialMomentum()
  {
    yield return new WaitForFixedUpdate();
    rb.AddForce(initialDirection * initialForce, ForceMode.Impulse);
    hasInitialMomentum = true;
  }

  void MaintainMinimumVelocity()
  {
    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);

    if (horizontalVelocity.magnitude < minimumVelocity)
    {
      Vector3 velocityDirection = horizontalVelocity.normalized;
      Vector3 newVelocity = velocityDirection * minimumVelocity;
      newVelocity.y = rb.velocity.y;
      rb.velocity = newVelocity;
    }
    else
    {
      rb.velocity *= velocityDamping;
    }
  }


  public void UpdateGravity(float input)
  {
    // W increases gravity, S decreases it
    float gravityDelta = input * gravityChangeSpeed * Time.deltaTime;
    currentGravity = Mathf.Clamp(currentGravity + gravityDelta, minGravity, maxGravity);
  }


  void OnGUI()
  {
    GUI.Label(new Rect(10, 10, 200, 20), $"Gravity: {currentGravity:F1}");
    if (isDisc)
    {
      float tiltAngle = transform.rotation.eulerAngles.z;
      if (tiltAngle > 180) tiltAngle -= 360;
      GUI.Label(new Rect(10, 30, 200, 20), $"Tilt Angle: {tiltAngle:F1}°");
    }
  }

  void OnDrawGizmos()
  {
    if (rb == null) return;

    // Draw velocity vector
    Gizmos.color = Color.blue;
    Gizmos.DrawLine(transform.position, transform.position + rb.velocity);

    // Draw current gravity magnitude
    Gizmos.color = Color.yellow;
    float gravityVisualization = currentGravity / maxGravity;
    Gizmos.DrawLine(transform.position, transform.position + Vector3.down * gravityVisualization * 2);
  }
}