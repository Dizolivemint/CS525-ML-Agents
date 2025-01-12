using Unity.MLAgents;
using UnityEngine;

public class BoundaryMetrics : MonoBehaviour
{
  private StatsRecorder statsRecorder;
  private int correctSequenceHits = 0;
  private int incorrectHits = 0;
  private float avgTimePerCorrectHit = 0;
  private float episodeStartTime;
  private float lastHitTime;
  private float lastDistanceToTarget = 0f;


  void Start()
  {
    statsRecorder = Academy.Instance.StatsRecorder;
    ResetMetrics();
  }

  public void StartNewEpisode()
  {
    // Record final stats for previous episode if any
    RecordEpisodeStats();
    ResetMetrics();
  }

  public void RecordDistanceToTarget(float distance)
  {
    statsRecorder.Add("Metrics/Distance To Target", distance);

    // Calculate and record progress (negative value means getting closer)
    float progress = distance - lastDistanceToTarget;
    statsRecorder.Add("Metrics/Distance Progress", progress);

    lastDistanceToTarget = distance;
  }

  public void RecordCorrectHit()
  {
    correctSequenceHits++;
    float timeSinceLastHit = Time.time - lastHitTime;
    avgTimePerCorrectHit = ((avgTimePerCorrectHit * (correctSequenceHits - 1)) + timeSinceLastHit) / correctSequenceHits;
    lastHitTime = Time.time;

    // Record immediate stats
    statsRecorder.Add("Metrics/Correct Hits", correctSequenceHits);
    statsRecorder.Add("Metrics/Time Between Hits", timeSinceLastHit);
    statsRecorder.Add("Metrics/Success Rate", GetSuccessRate());
  }

  public void RecordIncorrectHit()
  {
    incorrectHits++;
    statsRecorder.Add("Metrics/Incorrect Hits", incorrectHits);
    statsRecorder.Add("Metrics/Success Rate", GetSuccessRate());
  }

  private void RecordEpisodeStats()
  {
    if (correctSequenceHits + incorrectHits > 0)
    {
      float episodeTime = Time.time - episodeStartTime;
      statsRecorder.Add("Metrics/Episode Duration", episodeTime);
      statsRecorder.Add("Metrics/Average Time Per Hit", avgTimePerCorrectHit);
      statsRecorder.Add("Metrics/Final Success Rate", GetSuccessRate());
    }
  }

  private float GetSuccessRate()
  {
    int totalHits = correctSequenceHits + incorrectHits;
    return totalHits > 0 ? (float)correctSequenceHits / totalHits : 0f;
  }

  private void ResetMetrics()
  {
    correctSequenceHits = 0;
    incorrectHits = 0;
    avgTimePerCorrectHit = 0;
    episodeStartTime = Time.time;
    lastHitTime = Time.time;
    lastDistanceToTarget = 0f;
  }
}