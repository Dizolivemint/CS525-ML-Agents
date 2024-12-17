using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
  // Movement settings
  // Movement settings
  [Header("Movement")]
  public float rollSpeed = 200f;
  public float glideSpeed = 200f;
  public float initialForce = 1000f;
  public Vector3 initialDirection = Vector3.forward;

  // Tilt Settings
  [Header("Disc")]
  public float maxTiltAngle = 45f;
  public float tiltSpeed = 90f;
  public float tiltSmoothing = 5f;
  public float yawSpeed = 45f;  // Speed of turning
  public float maxPitchAngle = 90f; 
  private float targetPitch = 0f;
  private float targetRoll = 0f; // For steeper up/down angles

  // Gravity Settings
  [Header("Gravity")]
  public float gliderGravity = 0.1f;
  public float minGravity = 20f;
  public float maxGravity = 10000f;   // Increased from 20
  public float gravityChangeSpeed = 1000f;  // Increased for faster changes
  private float currentGravity;

  // Rolling Settings
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
    SetSphereForm();
    currentGravity = minGravity;
    targetRotation = transform.rotation;
    lastValidPosition = transform.position;
    StartCoroutine(ApplyInitialMomentum());
  }

  void FixedUpdate()
  {
    if (!hasInitialMomentum) return;

    // Store current position as potentially valid
    if (IsPositionSafe(transform.position))
    {
      lastValidPosition = transform.position;
    }

    if (isDisc)
    {
      HandleGliding();
      HandleDiscPhysics();
    }
    else
    {
      HandleRolling();
    }

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

  void Update()
{
    float horizontal = Input.GetAxisRaw("Horizontal");
    float vertical = Input.GetAxisRaw("Vertical");

    if (isDisc)
    {
        UpdateDiscRoll(horizontal);
        // Invert vertical for more intuitive controls (W = nose down, S = nose up)
        UpdateDiscPitch(-vertical);  // Inverted vertical for intuitive controls
        moveDirection = Vector3.zero;
    }
    else
    {
        moveDirection = new Vector3(horizontal, 0, 0).normalized;
    }

    // If W is held (positive vertical), increase gravity
    // Otherwise, instantly reset to minimum
    if (vertical > 0)
    {
        float gravityDelta = gravityChangeSpeed * Time.deltaTime;
        currentGravity = Mathf.Min(currentGravity + gravityDelta, maxGravity);
    }
    else
    {
        currentGravity = minGravity;
    }

    if (Input.GetKeyDown(KeyCode.Space))
    {
        Transform();
    }
}

void UpdateDiscPitch(float verticalInput)
{
    if (!isDisc) return;
    
    // More immediate response and larger range
    targetPitch = verticalInput * maxPitchAngle;
    Debug.Log($"Vertical Input: {verticalInput}, Setting Target Pitch: {targetPitch}");
}

void UpdateDiscRoll(float horizontalInput)
{
    if (!isDisc)
        return;
        
    // More immediate response
    targetRoll = -horizontalInput * maxTiltAngle;
    Debug.Log($"Horizontal Input: {horizontalInput}, Setting Target Roll: {targetRoll}");
}

void UpdateDiscRotation()
{
    // Get current yaw since we want to preserve it
    float currentYaw = transform.rotation.eulerAngles.y;
    
    // Combine pitch and roll with current yaw
    targetRotation = Quaternion.Euler(targetPitch, currentYaw, targetRoll);
}

void HandleDiscPhysics()
{
    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
    
    if (horizontalVelocity.magnitude > 0.1f)
    {
        // Get base rotation to face movement direction
        Quaternion baseRotation = Quaternion.LookRotation(horizontalVelocity.normalized);
        
        // Simply set the target X (pitch) and Z (roll) angles while keeping the Y (yaw) from base rotation
        float yaw = baseRotation.eulerAngles.y;
        targetRotation = Quaternion.Euler(targetPitch, yaw, targetRoll);
        
        // Apply smoothly
        Quaternion currentRotation = rb.rotation;
        Quaternion newRotation = Quaternion.Slerp(currentRotation, targetRotation, Time.fixedDeltaTime * tiltSmoothing);
        rb.MoveRotation(newRotation);

        // Handle pitch-based movement
        float pitchAngle = rb.rotation.eulerAngles.x;
        if (pitchAngle > 180) pitchAngle -= 360;
        float verticalForce = -Mathf.Sin(pitchAngle * Mathf.Deg2Rad) * currentGravity;
        rb.AddForce(Vector3.up * verticalForce, ForceMode.Acceleration);
    }

    // Handle roll-based movement
    float rollAngle = rb.rotation.eulerAngles.z;
    if (rollAngle > 180) rollAngle -= 360;

    float sidewaysForce = Mathf.Sin(rollAngle * Mathf.Deg2Rad) * currentGravity;
    rb.AddForce(Vector3.right * sidewaysForce, ForceMode.Acceleration);
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


  void UpdateGravity(float input)
  {
    // W increases gravity, S decreases it
    float gravityDelta = input * gravityChangeSpeed * Time.deltaTime;
    currentGravity = Mathf.Clamp(currentGravity + gravityDelta, minGravity, maxGravity);
  }

  void HandleGliding()
  {
    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
    float speed = horizontalVelocity.magnitude;

    // Get current roll angle for turning
    float rollAngle = transform.rotation.eulerAngles.z;
    if (rollAngle > 180) rollAngle -= 360;

    // Turn based on roll angle
    float turnForce = -rollAngle * 0.5f; // Adjust multiplier to control turn sensitivity
    transform.Rotate(Vector3.up * turnForce * Time.deltaTime);

    // Get pitch angle
    float pitchAngle = transform.rotation.eulerAngles.x;
    if (pitchAngle > 180) pitchAngle -= 360;

    // Dynamically adjust drag based on pitch angle
    // Only increase drag when pitching up (negative pitch angle)
    if (pitchAngle < 0)
    {
      // Convert pitch to 0-1 range and apply exponential scaling for more dramatic effect
      float pitchFactor = Mathf.Abs(pitchAngle) / maxPitchAngle;
      float extraDrag = pitchFactor * (pitchFactor / 10); // Square it for exponential effect, multiply for strength
      rb.drag = glidingDrag + extraDrag;
    }
    else
    {
      rb.drag = glidingDrag;
    }

    // Apply forward force in the direction we're facing
    Vector3 forwardDir = transform.forward;
    forwardDir.y = 0; // Keep it horizontal
    rb.AddForce(forwardDir.normalized * speed * 0.1f, ForceMode.Acceleration);

    float liftMultiplier = -pitchAngle / maxPitchAngle;
    float liftForce = speed * liftMultiplier;

    rb.AddForce(Vector3.down * currentGravity, ForceMode.Acceleration);
    rb.AddForce(Vector3.up * liftForce, ForceMode.Acceleration);

    // Orient the disc to face the direction of movement
    if (horizontalVelocity.magnitude > 0.1f)
    {
        // Get current up vector
        Vector3 currentUp = transform.up;
        
        // Calculate target forward direction from velocity
        Vector3 targetForward = horizontalVelocity.normalized;
        
        // Keep the up vector pointing upward (or slightly pitched based on current rotation)
        Vector3 worldUp = Vector3.up;
        float currentPitch = transform.rotation.eulerAngles.x;
        if (currentPitch > 180) currentPitch -= 360; // Normalize to -180 to 180
        
        // Apply pitch to the up vector
        Quaternion pitchRotation = Quaternion.Euler(currentPitch, 0, 0);
        Vector3 targetUp = pitchRotation * worldUp;
        
        // Create rotation that aligns with both forward and up directions
        Quaternion targetRotation = Quaternion.LookRotation(targetForward, targetUp);
        
        // Preserve the current roll
        float currentRoll = transform.rotation.eulerAngles.z;
        if (currentRoll > 180) currentRoll -= 360; // Normalize to -180 to 180
        
        // Apply roll after aligning with movement direction
        targetRotation *= Quaternion.Euler(0, 0, currentRoll);
        
        transform.rotation = targetRotation;
    }
    else
    {
        // When stationary, maintain pitch and roll, but reset yaw
        float currentPitch = transform.rotation.eulerAngles.x;
        float currentRoll = transform.rotation.eulerAngles.z;
        if (currentPitch > 180) currentPitch -= 360;
        if (currentRoll > 180) currentRoll -= 360;
        transform.rotation = Quaternion.Euler(currentPitch, 0, currentRoll);
    }
  }

  void Transform()
  {
    isDisc = !isDisc;

    if (isDisc)
    {
      SetDiscForm();
    }
    else
    {
      SetSphereForm();
    }

    rb.velocity *= velocityDamping;
  }

  void SetSphereForm()
  {
    meshFilter.mesh = sphereMesh;
    meshCollider.sharedMesh = sphereMesh;
    meshCollider.convex = true;
    transform.localScale = Vector3.one * sphereRadius * 2;
    rb.constraints = RigidbodyConstraints.FreezeRotation;
    rb.drag = rollingDrag;
    rb.angularDrag = 0f;

    rb.MoveRotation(Quaternion.identity);
    targetRotation = Quaternion.identity;
  }

void SetDiscForm()
{
    meshFilter.mesh = discMesh;
    meshCollider.sharedMesh = discMesh;
    meshCollider.convex = true;
    transform.localScale = new Vector3(discRadius * 4, discHeight, discRadius * 2);
    
    // Remove rotation constraints - we'll handle this in physics
    rb.constraints = RigidbodyConstraints.None;
    
    rb.drag = glidingDrag;
    rb.angularDrag = 2f;
    currentGravity = gliderGravity;
    
    targetRotation = transform.rotation;
    targetPitch = 0f;
    targetRoll = 0f;
}

  void OnCollisionEnter(Collision collision)
  {
    if (isDisc)
    {
      Vector3 currentEuler = rb.rotation.eulerAngles;
      float zAngle = currentEuler.z;
      if (zAngle > 180) zAngle -= 360;

      zAngle = Mathf.Clamp(zAngle, -maxTiltAngle, maxTiltAngle);
      rb.MoveRotation(Quaternion.Euler(0, 0, zAngle));
    }
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