using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using UnityEngine;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Demonstrations;

public class BoundarySequenceAgent : Agent
{
  [SerializeField] private PlayerController playerController;
  [SerializeField] private float correctReward = 10f;
  [SerializeField] private float incorrectPenalty = -5f;

  private Rigidbody rb;
  private readonly Boundary[] sequence = { Boundary.Front, Boundary.Left, Boundary.Right, Boundary.Back };
  private int currentIndex;
  private float lastBoundaryHitTime;

  private enum Boundary { None, Front, Left, Right, Back }

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
    rb = playerController.GetRigidbody();
    playerController.SetAgentControl(true);

    // Add demonstration recorder
    var recorder = gameObject.AddComponent<DemonstrationRecorder>();
    recorder.DemonstrationName = "agent_demo";
    recorder.Record = false;
  }

  public override void CollectObservations(VectorSensor sensor)
  {
    // Add observations needed for learning
    if (rb != null)
    {
      sensor.AddObservation(rb.velocity);
      sensor.AddObservation(rb.angularVelocity);
    }
    sensor.AddObservation(transform.position);
    sensor.AddObservation(currentIndex);
    sensor.AddObservation(Time.time - lastBoundaryHitTime);
  }

  public override void OnActionReceived(ActionBuffers actions)
  {
    if (rb == null || playerController == null) return;

    Vector2 input = new Vector2(
        actions.ContinuousActions[0],
        actions.ContinuousActions[1]
    );

    playerController.Move(input);
  }

  private void OnCollisionEnter(Collision collision)
  {
    if (!collision.gameObject.CompareTag("Boundary")) return;
    if (Time.time - lastBoundaryHitTime < 0.5f) return;

    lastBoundaryHitTime = Time.time;
    Boundary hitBoundary = GetBoundaryType(collision.transform);

    if (hitBoundary == sequence[currentIndex])
    {
      AddReward(correctReward);
      currentIndex++;
      if (currentIndex >= sequence.Length)
      {
        EndEpisode();
      }
    }
    else
    {
      AddReward(incorrectPenalty);
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

    float angle = Random.Range(0f, 360f);
    Vector3 direction = Quaternion.Euler(0, angle, 0) * Vector3.forward;
    rb.AddForce(direction * playerController.initialForce, ForceMode.Impulse);
  }

  private Boundary GetBoundaryType(Transform boundary)
  {
    Vector3 dirToCenter = (Vector3.zero - boundary.position).normalized;
    float dotForward = Vector3.Dot(dirToCenter, Vector3.forward);
    float dotRight = Vector3.Dot(dirToCenter, Vector3.right);

    if (Mathf.Abs(dotForward) > 0.7f)
      return dotForward > 0 ? Boundary.Back : Boundary.Front;
    if (Mathf.Abs(dotRight) > 0.7f)
      return dotRight > 0 ? Boundary.Left : Boundary.Right;

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
}