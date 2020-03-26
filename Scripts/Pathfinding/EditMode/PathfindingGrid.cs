using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[ExecuteAlways][System.Serializable]
public class PathfindingGrid : MonoBehaviour
{
    public static PathfindingGrid instance;

    [SerializeField] GameObject gridMeshPrefab;
    [SerializeField] GameObject blockedTilePrefab;
    [SerializeField] GameObject searchedTilePrefab;
    [SerializeField] GameObject waypointMarkerPrefab;

    [SerializeField] int numCells = 3;
    [SerializeField] int cellSize = 6;
    [SerializeField] int[] layers;
    [SerializeField] Color[] layerColors;

    [SerializeField] bool initialize = true;
    [SerializeField] bool update = true;

    private Transform meshLayers;
    private Transform blockedTiles;
    private Transform baseTransform;

    private void Awake()
    {
        Initialize();
    }
    private void Update()
    {
        if ( initialize )
            Initialize();

        if ( update )
        {
            update = false;

            UpdateChildren();
            UpdateMesh();
            UpdateBase();
        }
    }

    public void AddBlockedTile( Vector3 mousePosition )
    {
        Ray ray = HandleUtility.GUIPointToWorldRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            int col = Mathf.FloorToInt( hit.point.x / cellSize );
            int row = Mathf.FloorToInt( hit.point.z / cellSize );
            float x = col * cellSize + cellSize / 2;
            float y = row * cellSize + cellSize / 2;

            Transform blockedTile = Instantiate( blockedTilePrefab ).transform;
            blockedTile.transform.position = new Vector3( x , 1 , y );
            blockedTile.transform.localScale = new Vector3( cellSize , cellSize , 1 );
            blockedTile.transform.parent = blockedTiles;

            blockedTile.GetComponent<BlockedCell>().index = col + row * numCells;
        }
    }

    // Getters
    public int GetCellsAcross()
    {
        return numCells;
    }
    public int GetClustersAcross()
    {
        return numCells / layers[ 1 ];
    }
    public int GetCellSize()
    {
        return cellSize;
    }
    public int GetClusterSize()
    {
        return layers[ 1 ];
    }
    public int[] GetBlockedVerticeArray()
    {
        List<int> indices = new List<int>();

        for ( int i = 0; i < transform.GetChild( 1 ).childCount; i++ )
        {
            indices.Add( transform.GetChild( 1 ).GetChild( i ).GetComponent<BlockedCell>().index );
            indices.Add( transform.GetChild( 1 ).GetChild( i ).GetComponent<BlockedCell>().index + 1 );
            indices.Add( transform.GetChild( 1 ).GetChild( i ).GetComponent<BlockedCell>().index + numCells );
            indices.Add( transform.GetChild( 1 ).GetChild( i ).GetComponent<BlockedCell>().index + 1 + numCells );
        }

        return indices.ToArray();
    }
    public GameObject GetWaypointMarkerPrefab()
    {
        return waypointMarkerPrefab;
    }
    public GameObject GetSearchedTilePrefab()
    {
        return searchedTilePrefab;
    }

    private void Initialize()
    {
        initialize = false;
        update = false;

        meshLayers = transform.GetChild( 0 );
        blockedTiles = transform.GetChild( 1 );
        baseTransform = transform.GetChild( 2 );

        instance = this;
    }
    private void UpdateChildren()
    {
        for ( int i = 0; i < layers.Length; i++ )
        {
            Transform t = Instantiate( gridMeshPrefab ).transform;
            t.position = Vector3.zero;
            t.parent = meshLayers;
        }
    }
    private void UpdateMesh()
    {
        for ( int layer = 0; layer < layers.Length; layer++ )
        {
            MeshFilter filter = meshLayers.GetChild( layer ).GetComponent<MeshFilter>();

            var mesh = new Mesh();
            var verticies = new List<Vector3>();
            var indicies = new List<int>();

            int numOfIterations = numCells / layers[ layer ];

            for ( int i = 0; i <= numOfIterations; i++ )
            {
                verticies.Add( new Vector3( i * cellSize * layers[ layer ] , 1 + layer * 0.5f , 0 ) );
                verticies.Add( new Vector3( i * cellSize * layers[ layer ] , 1 + layer * 0.5f , numCells * cellSize ) );

                indicies.Add( 4 * i + 0 );
                indicies.Add( 4 * i + 1 );

                verticies.Add( new Vector3( 0 , 1 + layer * 0.5f , i * cellSize * layers[ layer ] ) );
                verticies.Add( new Vector3( numCells * cellSize , 1 + layer * 0.5f , i * cellSize * layers[ layer ] ) );

                indicies.Add( 4 * i + 2 );
                indicies.Add( 4 * i + 3 );
            }

            mesh.vertices = verticies.ToArray();
            mesh.SetIndices( indicies.ToArray() , MeshTopology.Lines , 0 );
            filter.mesh = mesh;

            MeshRenderer meshRenderer = meshLayers.GetChild( layer ).GetComponent<MeshRenderer>();
            meshRenderer.sharedMaterial = new Material( Shader.Find( "Sprites/Default" ) );
            meshRenderer.sharedMaterial.color = layerColors[ layer ];
        }
    }
    private void UpdateBase()
    {
        baseTransform.position = new Vector3(
            ( numCells / 2 ) * cellSize ,
            0 ,
            ( numCells / 2 ) * cellSize );
        baseTransform.localScale = new Vector3(
            numCells * cellSize ,
            0.5f ,
            numCells * cellSize );
    }
}
