using UnityEngine;

public class BoundarySetup : MonoBehaviour
{
  public float width = 1000f;
  public float height = 50f;
  public float depth = 1000f;
  public float bottomOffset = -100f;
  public Color wallColor = new Color(1, 0, 1, 0.3f);
  public Color edgeColor = Color.blue;
  public float lineWidth = 0.1f;

  [Header("Physics Settings")]
  public float bounciness = 0.8f;    // How bouncy the walls are (0-1)
  public float friction = 0.1f;       // How much friction the walls have
  private PhysicMaterial boundaryPhysicsMaterial;

  void Start()
  {
    GameObject player = GameObject.FindGameObjectWithTag("Player");
    if (player != null)
    {
      Vector3 playerPos = player.transform.position;
      transform.position = playerPos;
    }

    CreatePhysicsMaterial();
    CreateBoundaries();
    CreateEdges();
  }

  void CreatePhysicsMaterial()
  {
    // Create a new physics material
    boundaryPhysicsMaterial = new PhysicMaterial("BoundaryMaterial");
    boundaryPhysicsMaterial.bounciness = bounciness;
    boundaryPhysicsMaterial.dynamicFriction = friction;
    boundaryPhysicsMaterial.frictionCombine = PhysicMaterialCombine.Minimum;
    boundaryPhysicsMaterial.bounceCombine = PhysicMaterialCombine.Maximum;
  }

  void CreateBoundaries()
  {
    // Bottom and Top
    CreateVisibleWall(new Vector3(0, bottomOffset, 0), new Vector3(width, 1, depth), "Bottom");
    CreateVisibleWall(new Vector3(0, height, 0), new Vector3(width, 1, depth), "Top");

    // Training sequence walls
    CreateVisibleWall(
        new Vector3(-width / 2, height / 2 + bottomOffset / 2, 0),
        new Vector3(1, height + Mathf.Abs(bottomOffset), depth),
        "Left"
    );

    CreateVisibleWall(
        new Vector3(width / 2, height / 2 + bottomOffset / 2, 0),
        new Vector3(1, height + Mathf.Abs(bottomOffset), depth),
        "Right"
    );

    CreateVisibleWall(
        new Vector3(0, height / 2 + bottomOffset / 2, -depth / 2),
        new Vector3(width, height + Mathf.Abs(bottomOffset), 1),
        "Back"
    );

    CreateVisibleWall(
        new Vector3(0, height / 2 + bottomOffset / 2, depth / 2),
        new Vector3(width, height + Mathf.Abs(bottomOffset), 1),
        "Front"
    );
  }

  void CreateVisibleWall(Vector3 position, Vector3 size, string wallType)
  {
    GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
    Rigidbody rb = wall.AddComponent<Rigidbody>();
    rb.isKinematic = true;
    rb.useGravity = false;
    rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

    wall.transform.parent = transform;
    wall.transform.position = position;
    wall.transform.localScale = size;

    // Set name and tag for ML-Agent detection
    wall.name = wallType + "Wall";
    wall.tag = "Boundary";

    // Create a custom layer for boundaries if it doesn't exist
    if (LayerMask.NameToLayer("Boundary") == -1)
    {
      // Note: You'll need to manually create a layer named "Boundary" in Unity
      Debug.LogWarning("Please create a layer named 'Boundary' in Unity's Layer settings");
    }
    wall.layer = LayerMask.NameToLayer("Boundary");

    // Add physics material to the collider
    Collider wallCollider = wall.GetComponent<Collider>();
    wallCollider.material = boundaryPhysicsMaterial;

    // Set up the visual material with different colors for each wall type
    Renderer renderer = wall.GetComponent<Renderer>();
    Material mat = new Material(Shader.Find("Standard"));

    // Assign different colors to help identify walls
    switch (wallType)
    {
      case "Front":
        mat.color = new Color(1, 0, 0, 0.3f); // Red
        break;
      case "Left":
        mat.color = new Color(0, 1, 0, 0.3f); // Green
        break;
      case "Right":
        mat.color = new Color(0, 0, 1, 0.3f); // Blue
        break;
      case "Back":
        mat.color = new Color(1, 1, 0, 0.3f); // Yellow
        break;
      default:
        mat.color = wallColor;
        break;
    }

    renderer.material = mat;
  }
  void CreateEdges()
  {
    DrawEdge(new Vector3(-width / 2, bottomOffset, -depth / 2), new Vector3(width / 2, bottomOffset, -depth / 2));
    DrawEdge(new Vector3(-width / 2, bottomOffset, depth / 2), new Vector3(width / 2, bottomOffset, depth / 2));
    DrawEdge(new Vector3(-width / 2, bottomOffset, -depth / 2), new Vector3(-width / 2, bottomOffset, depth / 2));
    DrawEdge(new Vector3(width / 2, bottomOffset, -depth / 2), new Vector3(width / 2, bottomOffset, depth / 2));

    DrawEdge(new Vector3(-width / 2, height, -depth / 2), new Vector3(width / 2, height, -depth / 2));
    DrawEdge(new Vector3(-width / 2, height, depth / 2), new Vector3(width / 2, height, depth / 2));
    DrawEdge(new Vector3(-width / 2, height, -depth / 2), new Vector3(-width / 2, height, depth / 2));
    DrawEdge(new Vector3(width / 2, height, -depth / 2), new Vector3(width / 2, height, depth / 2));

    DrawEdge(new Vector3(-width / 2, bottomOffset, -depth / 2), new Vector3(-width / 2, height, -depth / 2));
    DrawEdge(new Vector3(width / 2, bottomOffset, -depth / 2), new Vector3(width / 2, height, -depth / 2));
    DrawEdge(new Vector3(-width / 2, bottomOffset, depth / 2), new Vector3(-width / 2, height, depth / 2));
    DrawEdge(new Vector3(width / 2, bottomOffset, depth / 2), new Vector3(width / 2, height, depth / 2));
  }

  void DrawEdge(Vector3 start, Vector3 end)
  {
    GameObject edge = new GameObject("Edge");
    edge.transform.parent = transform;

    LineRenderer line = edge.AddComponent<LineRenderer>();
    line.material = new Material(Shader.Find("Sprites/Default"));
    line.startColor = edgeColor;
    line.endColor = edgeColor;
    line.startWidth = lineWidth;
    line.endWidth = lineWidth;
    line.SetPosition(0, start);
    line.SetPosition(1, end);
  }
}