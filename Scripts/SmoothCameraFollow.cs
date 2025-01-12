using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmoothCameraFollow : MonoBehaviour
{
  [Header("Target Settings")]
  [SerializeField] private Transform target;
  [SerializeField] private Vector3 offset = new Vector3(0, 5, -5);
  [SerializeField] private Vector3 trainingOffset = new Vector3(0, 15, -30);
  [SerializeField] private bool isTrainingView = false;

  [Header("Movement Settings")]
  [SerializeField] private float smoothSpeed = 1f;
  [SerializeField] private float lookAheadFactor = 2f;

  [Header("Terrain Following")]
  [SerializeField] private float minHeightAboveTerrain = 2f;
  [SerializeField] private LayerMask terrainLayer;

  private Vector3 velocity = Vector3.zero;

  private void Start()
  {
    // Auto-detect if we're in training mode
    var agent = target.GetComponent<BoundarySequenceAgent>();
    isTrainingView = (agent != null && agent.enabled);
  }
  private void LateUpdate()
  {
    if (target == null) return;

    Vector3 currentOffset = isTrainingView ? trainingOffset : offset;

    // Calculate base position
    Vector3 desiredPosition = target.position + currentOffset;

    // Add look-ahead based on player velocity
    Rigidbody targetRb = target.GetComponent<Rigidbody>();
    if (targetRb != null)
    {
      desiredPosition += targetRb.velocity * lookAheadFactor * Time.deltaTime;
    }

    // Check terrain height
    RaycastHit hit;
    if (Physics.Raycast(desiredPosition, Vector3.down, out hit, 100f, terrainLayer))
    {
      float desiredHeight = hit.point.y + minHeightAboveTerrain + currentOffset.y;
      desiredPosition.y = Mathf.Max(desiredPosition.y, desiredHeight);
    }

    // Smoothly move camera
    transform.position = Vector3.SmoothDamp(
        transform.position,
        desiredPosition,
        ref velocity,
        1f / smoothSpeed
    );

    // Make camera look at target
    transform.LookAt(target);

    // Apply tilt constraint
    if (isTrainingView)
    {
      ApplyTiltConstraint();
    }
  }

  private void ApplyTiltConstraint()
  {
    Vector3 currentRotation = transform.rotation.eulerAngles;
    if (currentRotation.x > 180f) currentRotation.x -= 360f;
    currentRotation.x = Mathf.Clamp(currentRotation.x, -20f, 20f);
    transform.rotation = Quaternion.Euler(currentRotation);
  }
}
