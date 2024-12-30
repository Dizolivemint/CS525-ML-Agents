using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class BoundarySequenceAgent : Agent
{
  [Header("References")]
  [SerializeField] private PlayerController playerController;
  private Rigidbody rb;

  [Header("Boundary Detection")]
  public float boundaryCooldown = 0.5f;
  public LayerMask boundaryLayer;

  [Header("Reward Settings")]
  public float correctSequenceReward = 10f;
  public float wrongSequencePenalty = -5f;
  public float velocityReward = 0.01f;
  public float completionBonus = 50f;

  [Header("Movement Rewards")]
  public float directionAlignmentReward = 0.1f;
  public float efficientGravityReward = 0.05f;
  public float optimalSpeedReward = 0.2f;
  public float targetSpeedThreshold = 50f;

  private BoundaryMetrics metrics;
  private Vector3 lastPosition;
  private float stuckTimeout = 5f;
  private float lastMoveTime;
  private float minPositionChange = 0.1f;

  // Sequence tracking
  private enum Boundary { None, Front, Left, Right, Back }
  private Boundary[] targetSequence = {
        Boundary.Front,
        Boundary.Left,
        Boundary.Right,
        Boundary.Back
    };
  private int currentSequenceIndex = 0;
  private float lastBoundaryHitTime = 0f;

  void Start()
  {
    Debug.Log("Agent Started");
  }


  public override void Initialize()
  {
    base.Initialize();
    metrics = gameObject.AddComponent<BoundaryMetrics>();
    rb = playerController.GetComponent<Rigidbody>();
    lastPosition = transform.position;
    lastMoveTime = Time.time;
  }

protected override void Awake() {
    base.Awake();
    var behaviorParameters = gameObject.GetComponent<Unity.MLAgents.Policies.BehaviorParameters>();
    if (behaviorParameters != null) {
        Debug.Log($"Behavior Name: {behaviorParameters.BehaviorName}");
        Debug.Log($"Behavior Type: {behaviorParameters.BehaviorType}");
        Debug.Log($"Model: {behaviorParameters.Model}");
    } else {
        Debug.LogError("No BehaviorParameters found!");
    }
}
  void Update()
  { 
    float distanceMoved = Vector3.Distance(transform.position, lastPosition);
    if (distanceMoved > minPositionChange)
    {
      lastMoveTime = Time.time;
      lastPosition = transform.position;
    }
    else if (Time.time - lastMoveTime > stuckTimeout)
    {
      Debug.Log("Position stuck - resetting");
      OnEpisodeBegin();
    }
  }

  private Vector3 GetTargetBoundaryDirection()
  {
    switch (targetSequence[currentSequenceIndex])
    {
      case Boundary.Front: return Vector3.forward;
      case Boundary.Back: return Vector3.back;
      case Boundary.Left: return Vector3.left;
      case Boundary.Right: return Vector3.right;
      default: return Vector3.zero;
    }
  }

  public override void OnActionReceived(ActionBuffers actions)
    {
        Vector3 targetDir = GetTargetBoundaryDirection();

        // Apply gravity control directly
        float gravityControl = actions.ContinuousActions[0];

        playerController.currentGravity = Mathf.Lerp(
            playerController.minGravity,
            playerController.maxGravity,
            (gravityControl + 1f) / 2f  // Convert from [-1,1] to [0,1]
        );

        // Calculate alignment between velocity and target direction
        float alignment = Vector3.Dot(rb.velocity.normalized, targetDir);

        // Reward for aligning movement with target
        if (alignment > 0.7f)
        {
            AddReward(directionAlignmentReward * alignment);

            // Additional reward for using gravity efficiently when aligned
            if (playerController.currentGravity > playerController.minGravity && rb.velocity.magnitude < targetSpeedThreshold)
            {
                AddReward(efficientGravityReward);
            }

            // Reward for maintaining optimal speed range
            if (rb.velocity.magnitude >= targetSpeedThreshold)
            {
                AddReward(optimalSpeedReward);
            }
        }

        // Check if we're on a slope facing our target
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, 2f))
        {
            float slopeAlignment = Vector3.Dot(hit.normal, -targetDir);
            if (slopeAlignment > 0.3f && playerController.currentGravity > playerController.minGravity)
            {
                AddReward(efficientGravityReward * slopeAlignment);
            }
        }
    }

  void OnCollisionEnter(Collision collision)
  {
    if (!collision.gameObject.CompareTag("Boundary")) return;
    if (Time.time - lastBoundaryHitTime < boundaryCooldown) return;

    lastBoundaryHitTime = Time.time;
    Boundary hitBoundary = GetBoundaryType(collision.transform);

    if (hitBoundary == targetSequence[currentSequenceIndex])
    {
      AddReward(correctSequenceReward);
      currentSequenceIndex++;
      metrics.RecordCorrectHit();

      if (currentSequenceIndex >= targetSequence.Length)
      {
        AddReward(completionBonus);
        EndEpisode();
      }
    }
    else
    {
      AddReward(wrongSequencePenalty);
      metrics.RecordIncorrectHit();
      EndEpisode();
    }
  }

  public override void OnEpisodeBegin()
  {
    base.OnEpisodeBegin();
    Debug.Log("Episode Started");

    transform.localPosition = Vector3.zero;
    transform.rotation = Quaternion.identity;
    rb.velocity = Vector3.zero;

    currentSequenceIndex = 0;
    lastBoundaryHitTime = 0f;

    float randomAngle = Random.Range(0f, 360f);
    Vector3 randomDirection = Quaternion.Euler(0, randomAngle, 0) * Vector3.forward;
    rb.AddForce(randomDirection * playerController.initialForce, ForceMode.Impulse);
    metrics.StartNewEpisode();
  }

  public override void CollectObservations(VectorSensor sensor)
  {
    base.CollectObservations(sensor);
    Debug.Log("Collecting Observations");

    sensor.AddObservation(transform.localPosition);
    sensor.AddObservation(rb.velocity);
    sensor.AddObservation(playerController.currentGravity);

    AddBoundaryDistances(sensor);
    sensor.AddObservation(currentSequenceIndex);
  }

  private void AddBoundaryDistances(VectorSensor sensor)
  {
    Vector3[] directions = {
            Vector3.forward, Vector3.back,
            Vector3.left, Vector3.right,
            Vector3.up, Vector3.down
        };

    foreach (Vector3 dir in directions)
    {
      RaycastHit hit;
      if (Physics.Raycast(transform.position, dir, out hit, 1000f, boundaryLayer))
      {
        sensor.AddObservation(hit.distance);
        sensor.AddObservation(Vector3.Dot(rb.velocity.normalized, hit.normal));
      }
      else
      {
        sensor.AddObservation(1000f);
        sensor.AddObservation(0f);
      }
    }
  }

  private Boundary GetBoundaryType(Transform boundary)
  {
    Vector3 dirToCenter = (Vector3.zero - boundary.position).normalized;

    float dot = Vector3.Dot(dirToCenter, Vector3.forward);
    if (Mathf.Abs(dot) > 0.7f)
      return dot > 0 ? Boundary.Back : Boundary.Front;

    dot = Vector3.Dot(dirToCenter, Vector3.right);
    if (Mathf.Abs(dot) > 0.7f)
      return dot > 0 ? Boundary.Left : Boundary.Right;

    return Boundary.None;
  }

  public override void Heuristic(in ActionBuffers actionsOut)
  {
    var continuousActionsOut = actionsOut.ContinuousActions;
    continuousActionsOut[0] = Input.GetAxis("Vertical");
  }
}