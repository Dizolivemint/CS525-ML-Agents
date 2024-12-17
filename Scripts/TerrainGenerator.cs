using UnityEngine;
using System.Collections.Generic;

public class InfiniteTerrainGenerator : MonoBehaviour 
{
    [Header("Terrain Settings")]
    [SerializeField] private Transform player;
    [SerializeField] private int viewDistance = 3;
    [SerializeField] private float chunkSize = 20f;
    [SerializeField] private float heightScale = 2f; // Reduced from 20 to 2
    [SerializeField] private float noiseScale = 50f;
    [SerializeField] private float baseHeight = 0f; 
    
    [Header("Physics Settings")]
    [SerializeField] private PhysicMaterial terrainPhysicsMaterial;
    
    private Dictionary<Vector2Int, GameObject> terrainChunks = new Dictionary<Vector2Int, GameObject>();
    private Vector2Int currentPlayerChunk;
    
    private void Start()
    {
        // Generate initial chunks on start
        Vector2Int startChunk = WorldToChunkPosition(player.position);
        UpdateTerrainChunks(startChunk);
        
        if (terrainPhysicsMaterial == null)
        {
            terrainPhysicsMaterial = new PhysicMaterial
            {
                name = "TerrainPhysics",
                bounciness = 0.1f,
                staticFriction = 0.6f,
                dynamicFriction = 0.6f,
                frictionCombine = PhysicMaterialCombine.Average,
                bounceCombine = PhysicMaterialCombine.Average
            };
        }
    }
    
    private void Update()
    {
        if (player == null) return;
        
        Vector2Int newPlayerChunk = WorldToChunkPosition(player.position);
        if (newPlayerChunk != currentPlayerChunk)
        {
            UpdateTerrainChunks(newPlayerChunk);
            currentPlayerChunk = newPlayerChunk;
        }
    }

    private Vector2Int WorldToChunkPosition(Vector3 worldPosition)
    {
        return new Vector2Int(
            Mathf.FloorToInt(worldPosition.x / chunkSize),
            Mathf.FloorToInt(worldPosition.z / chunkSize)
        );
    }
    
    private void UpdateTerrainChunks(Vector2Int centerChunk)
    {
        HashSet<Vector2Int> chunksToKeep = new HashSet<Vector2Int>();
        
        // Generate or keep chunks within view distance
        for (int x = -viewDistance; x <= viewDistance; x++)
        {
            for (int z = -viewDistance; z <= viewDistance; z++)
            {
                Vector2Int chunkPosition = centerChunk + new Vector2Int(x, z);
                chunksToKeep.Add(chunkPosition);
                
                if (!terrainChunks.ContainsKey(chunkPosition))
                {
                    CreateTerrainChunk(chunkPosition);
                }
            }
        }
        
        // Remove chunks outside view distance
        List<Vector2Int> chunksToRemove = new List<Vector2Int>();
        foreach (var chunk in terrainChunks)
        {
            if (!chunksToKeep.Contains(chunk.Key))
            {
                chunksToRemove.Add(chunk.Key);
            }
        }
        
        foreach (var chunk in chunksToRemove)
        {
            DestroyTerrainChunk(chunk);
        }
    }
    
    private void CreateTerrainChunk(Vector2Int chunkPosition)
    {
        GameObject chunk = new GameObject($"Chunk_{chunkPosition.x}_{chunkPosition.y}");
        chunk.transform.parent = transform;
        chunk.layer = gameObject.layer;
        
        MeshFilter meshFilter = chunk.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = chunk.AddComponent<MeshRenderer>();
        MeshCollider meshCollider = chunk.AddComponent<MeshCollider>();
        
        meshCollider.sharedMaterial = terrainPhysicsMaterial;
        meshCollider.convex = false;
        meshCollider.isTrigger = false;
        
        Mesh mesh = GenerateTerrainMesh(chunkPosition);
        meshFilter.sharedMesh = mesh;
        meshCollider.sharedMesh = mesh;
        
        chunk.transform.position = new Vector3(
            chunkPosition.x * chunkSize,
            0,
            chunkPosition.y * chunkSize
        );
        
        Material terrainMaterial = new Material(Shader.Find("Standard"));
        terrainMaterial.color = new Color(0.3f, 0.5f, 0.2f);
        meshRenderer.material = terrainMaterial;
        
        terrainChunks.Add(chunkPosition, chunk);
    }
    
    private void DestroyTerrainChunk(Vector2Int chunkPosition)
    {
        GameObject chunk = terrainChunks[chunkPosition];
        terrainChunks.Remove(chunkPosition);
        Destroy(chunk);
    }
    
    private Mesh GenerateTerrainMesh(Vector2Int chunkPosition)
    {
        Mesh mesh = new Mesh();
        
        int resolution = 20;
        float step = chunkSize / (resolution - 1);
        
        Vector3[] vertices = new Vector3[resolution * resolution];
        int[] triangles = new int[(resolution - 1) * (resolution - 1) * 6];
        Vector2[] uvs = new Vector2[vertices.Length];
        
        for (int z = 0; z < resolution; z++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float worldX = (chunkPosition.x * chunkSize) + (x * step);
                float worldZ = (chunkPosition.y * chunkSize) + (z * step);
                
                float height = GenerateHeight(worldX, worldZ);
                vertices[z * resolution + x] = new Vector3(x * step, height, z * step);
                uvs[z * resolution + x] = new Vector2(x / (float)(resolution - 1), z / (float)(resolution - 1));
            }
        }
        
        int triangleIndex = 0;
        for (int z = 0; z < resolution - 1; z++)
        {
            for (int x = 0; x < resolution - 1; x++)
            {
                int vertexIndex = z * resolution + x;
                
                triangles[triangleIndex] = vertexIndex;
                triangles[triangleIndex + 1] = vertexIndex + resolution;
                triangles[triangleIndex + 2] = vertexIndex + 1;
                
                triangles[triangleIndex + 3] = vertexIndex + 1;
                triangles[triangleIndex + 4] = vertexIndex + resolution;
                triangles[triangleIndex + 5] = vertexIndex + resolution + 1;
                
                triangleIndex += 6;
            }
        }
        
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = uvs;
        mesh.RecalculateNormals();
        
        return mesh;
    }
    
    private float GenerateHeight(float x, float z)
    {
        float xCoord = x / noiseScale;
        float zCoord = z / noiseScale;
        
        // Generate smaller variations and add to base height
        float noise = Mathf.PerlinNoise(xCoord, zCoord);
        return baseHeight + (noise * heightScale);
    }
}