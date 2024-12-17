using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SmoothCameraFollow : MonoBehaviour
{
    [Header("Target Settings")]
    [SerializeField] private Transform target;
    [SerializeField] private Vector3 offset = new Vector3(0, 5, -5);
    
    [Header("Movement Settings")]
    [SerializeField] private float smoothSpeed = 1f;
    [SerializeField] private float lookAheadFactor = 2f;
    
    [Header("Terrain Following")]
    [SerializeField] private float minHeightAboveTerrain = 2f;
    [SerializeField] private LayerMask terrainLayer;
    
    private Vector3 velocity = Vector3.zero;
    
    private void LateUpdate()
    {
        if (target == null) return;
        
        // Calculate base position
        Vector3 desiredPosition = target.position + offset;
        
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
            float desiredHeight = hit.point.y + minHeightAboveTerrain + offset.y;
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
    }
}
