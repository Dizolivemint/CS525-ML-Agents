using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Demonstrations;
using System.Collections.Generic;

[RequireComponent(typeof(BoundaryMetrics))]
public class BoundarySequenceAgent : Agent
{
  [SerializeField] private PlayerController playerController;
  [SerializeField] private float correctReward = 15f;
  [SerializeField] private float incorrectPenalty = -1f;
  [SerializeField] private float distanceRewardScale = 0.5f;
  [SerializeField] private float inputSmoothTime = 0.2f;
  private Vector3 lastPosition;
  private Transform currentTargetBoundary;
  [SerializeField] private float episodeTimeLimit = 90f; // Seconds
  [SerializeField] private float timePenaltyScale = -0.0003f;
  private float episodeStartTime;

  private Rigidbody rb;
  private readonly Boundary[] sequence = { Boundary.Front, Boundary.Left, Boundary.Right, Boundary.Back };
  private int currentIndex;
  private float lastBoundaryHitTime;

  private enum Boundary { None, Front, Left, Right, Back }

  private Dictionary<Boundary, Transform> _boundaryTransforms;
  private BoundaryMetrics metrics;

  // Input smoothing
  private Vector2 currentInput;
  private Vector2 inputVelocity;

  [SerializeField] private Camera agentCamera;
  private void Start()
  {
    // If camera not assigned in inspector, try to find it
    if (agentCamera == null)
    {
      Debug.LogError("No camera assigned or found for BoundarySequenceAgent!");
    }
  }

  private void OnValidate()
  {
    if (playerController == null)
    {
      Debug.LogError("PlayerController reference is required!");
    }
  }

  public override void Initialize()
  {
    base.Initialize();
    if (playerController == null)
    {
      playerController = GetComponent<PlayerController>();
    }
    metrics = GetComponent<BoundaryMetrics>();
    rb = playerController.GetRigidbody();
    playerController.SetAgentControl(true);
    currentInput = Vector2.zero;
    inputVelocity = Vector2.zero;

    // Add demonstration recorder
    var recorder = gameObject.AddComponent<DemonstrationRecorder>();
    recorder.DemonstrationName = "agent_demo";
    recorder.Record = false;
  }

  public override void CollectObservations(VectorSensor sensor)
  {
    if (rb == null) return;

  /*
    // Get camera's coordinate system
    Vector3 cameraForward = agentCamera.transform.forward;
    cameraForward.y = 0;
    cameraForward.Normalize();
    Vector3 cameraRight = agentCamera.transform.right;
    cameraRight.y = 0;
    cameraRight.Normalize();

    // Agent position (3) - keep as is since it's in local space
    sensor.AddObservation(transform.localPosition);

    // Agent velocity (3) - keep as is for physics accuracy
    sensor.AddObservation(rb.velocity);

    // Terrain gradient at agent position (2)
    Vector3 terrainNormal = GetTerrainNormalAtPosition(transform.position);
    sensor.AddObservation(terrainNormal.x);
    sensor.AddObservation(terrainNormal.z);

    // Current boundary index (1)
    sensor.AddObservation(currentIndex);

    // Distance to next target boundary in camera space (2)
    Vector3 targetPosition = GetTargetBoundaryPosition();
    Vector3 directionToTarget = (targetPosition - transform.position);
    float forwardAmount = Vector3.Dot(directionToTarget, cameraForward);
    float rightAmount = Vector3.Dot(directionToTarget, cameraRight);
    sensor.AddObservation(forwardAmount);
    sensor.AddObservation(rightAmount);

    // Time since last hit (1)
    sensor.AddObservation(Time.time - lastBoundaryHitTime);

    if (currentTargetBoundary == null) return;

    // Distance to target boundary (1)
    float distanceToTarget = Vector3.Distance(transform.position, currentTargetBoundary.position);
    sensor.AddObservation(distanceToTarget);

    // Current velocity (3 + 1)
    sensor.AddObservation(rb.velocity.normalized);
    sensor.AddObservation(rb.velocity.magnitude);

*/
    // Simplified essential observations
    sensor.AddObservation(transform.localPosition);
    sensor.AddObservation(rb.velocity.normalized);
    sensor.AddObservation(currentIndex);
    
    if (currentTargetBoundary != null)
    {
        Vector3 directionToTarget = (currentTargetBoundary.position - transform.position).normalized;
        sensor.AddObservation(directionToTarget);
        sensor.AddObservation(Vector3.Distance(transform.position, currentTargetBoundary.position));
    }
  }
  private Vector3 GetTerrainNormalAtPosition(Vector3 position)
  {
    RaycastHit hit;
    if (Physics.Raycast(position + Vector3.up, Vector3.down, out hit, 2f, LayerMask.GetMask("Terrain")))
    {
      return hit.normal;
    }
    return Vector3.up;
  }

  private Vector3 GetTargetBoundaryPosition()
  {
    // Implementation to return position of next target boundary
    // Based on currentIndex and sequence array
    return Vector3.zero; // Placeholder - implement actual logic
  }

  public override void OnActionReceived(ActionBuffers actions)
  {
    if (rb == null || playerController == null) return;

    Vector2 targetInput = new Vector2(
        actions.ContinuousActions[0],
        actions.ContinuousActions[1]
    );

    // Smoothly transition to new input values
    currentInput = Vector2.SmoothDamp(
      currentInput, 
      targetInput, 
      ref inputVelocity, 
      inputSmoothTime
    );

    playerController.Move(currentInput);

    if (currentTargetBoundary != null)
    {
      float oldDistance = Vector3.Distance(lastPosition, currentTargetBoundary.position);
      float newDistance = Vector3.Distance(transform.position, currentTargetBoundary.position);
      float distanceReward = (oldDistance - newDistance) * distanceRewardScale;
      AddReward(distanceReward);

      if (metrics != null)
      {
        metrics.RecordDistanceToTarget(newDistance);
      }
    }

    lastPosition = transform.position;

    AddReward(timePenaltyScale);

    // End episode if taking too long
    if (Time.time - episodeStartTime > episodeTimeLimit)
    {
      AddReward(-1f); // Large penalty for timeout
      EndEpisode();
    }
  }

  private void OnCollisionEnter(Collision collision)
  {
    if (!collision.gameObject.CompareTag("Boundary")) return;
    if (Time.time - lastBoundaryHitTime < 0.5f) return;

    lastBoundaryHitTime = Time.time;
    Boundary hitBoundary = GetBoundaryType(collision.transform);

    // if (hitBoundary == sequence[currentIndex])
    // {
    //   AddReward(correctReward);
    //   if (metrics != null) metrics.RecordCorrectHit();
    //   currentIndex++;
    //   if (currentIndex >= sequence.Length)
    //   {
    //     EndEpisode();
    //   }
    // }
    // Only require that the first boundary in the sequence is hit
    if (hitBoundary == sequence[0])
    {
        AddReward(correctReward);
        EndEpisode();
    }
    else
    {
      AddReward(incorrectPenalty);
      if (metrics != null) metrics.RecordIncorrectHit();
      EndEpisode();
    }
  }

  public override void OnEpisodeBegin()
  {
    if (rb == null && playerController != null)
    {
      rb = playerController.GetRigidbody();
    }

    if (rb == null)
    {
      Debug.LogError("Rigidbody not found!");
      return;
    }

    currentIndex = 0;
    transform.localPosition = Vector3.zero;
    transform.rotation = Quaternion.identity;
    rb.velocity = Vector3.zero;
    rb.angularVelocity = Vector3.zero;
    currentInput = Vector2.zero;
    inputVelocity = Vector2.zero;

    float angle = Random.Range(0f, 360f);
    Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
    rb.AddForce(direction * playerController.initialForce, ForceMode.Impulse);

    lastPosition = transform.position;
    currentTargetBoundary = GetCurrentTargetBoundary();

    episodeStartTime = Time.time;
    metrics.StartNewEpisode();
  }

  private Boundary GetBoundaryType(Transform boundary)
  {
    // Get the closest point on the boundary to the agent
    Vector3 agentPos = transform.position;
    Vector3 closestPoint = boundary.GetComponent<Collider>().ClosestPoint(agentPos);

    // Calculate direction using the closest point instead of center
    Vector3 dirToWall = (closestPoint - agentPos).normalized;
    float dotForward = Vector3.Dot(dirToWall, Vector3.forward);
    float dotRight = Vector3.Dot(dirToWall, Vector3.right);

    // Use tighter bounds for more accurate detection
    if (Mathf.Abs(dotForward) > 0.8f)
      return dotForward > 0 ? Boundary.Front : Boundary.Back;
    if (Mathf.Abs(dotRight) > 0.8f)
      return dotRight > 0 ? Boundary.Right : Boundary.Left;

    return Boundary.None;
  }

  public override void Heuristic(in ActionBuffers actionsOut)
  {
    var continuousActions = actionsOut.ContinuousActions;
    continuousActions[0] = Input.GetAxis("Horizontal");
    continuousActions[1] = Input.GetAxis("Vertical");
  }

  public int GetCurrentIndex()
  {
    return currentIndex;
  }

  private Transform GetCurrentTargetBoundary()
  {
    // Cache references to boundaries on first call
    if (_boundaryTransforms == null)
    {
      _boundaryTransforms = new Dictionary<Boundary, Transform>();

      // Find all boundary objects
      GameObject[] boundaryObjects = GameObject.FindGameObjectsWithTag("Boundary");
      foreach (GameObject boundaryObj in boundaryObjects)
      {
        Boundary boundaryType = GetBoundaryType(boundaryObj.transform);
        if (boundaryType != Boundary.None)
        {
          _boundaryTransforms[boundaryType] = boundaryObj.transform;
        }
      }
    }

    // Return the transform for the current target in the sequence
    if (currentIndex < sequence.Length && _boundaryTransforms.ContainsKey(sequence[currentIndex]))
    {
      return _boundaryTransforms[sequence[currentIndex]];
    }

    return null;
  }
}