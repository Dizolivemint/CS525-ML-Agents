using UnityEngine;

public class BoundaryVisualizer : MonoBehaviour
{
    private BoundarySequenceAgent agent;
    private BoundarySetup boundaries;
    private readonly Color targetColor = Color.green;
    private readonly Color defaultColor = new Color(1, 0, 1, 0.3f);
    
    void Start()
    {
        agent = FindObjectOfType<BoundarySequenceAgent>();
        boundaries = FindObjectOfType<BoundarySetup>();
    }

    void Update()
    {
        GameObject[] walls = GameObject.FindGameObjectsWithTag("Boundary");
        foreach (GameObject wall in walls)
        {
            Renderer renderer = wall.GetComponent<Renderer>();
            bool isTargetWall = IsNextTarget(wall.transform);
            renderer.material.color = isTargetWall ? targetColor : defaultColor;
        }
    }

    private bool IsNextTarget(Transform boundary)
    {
        Vector3 dirToCenter = (Vector3.zero - boundary.position).normalized;
        float dotForward = Vector3.Dot(dirToCenter, Vector3.forward);
        float dotRight = Vector3.Dot(dirToCenter, Vector3.right);
        
        if (Mathf.Abs(dotForward) > 0.7f)
            return dotForward > 0 ? (agent.GetCurrentIndex() == 3) : (agent.GetCurrentIndex() == 0);
        if (Mathf.Abs(dotRight) > 0.7f)
            return dotRight > 0 ? (agent.GetCurrentIndex() == 1) : (agent.GetCurrentIndex() == 2);
            
        return false;
    }
}