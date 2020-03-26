using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

public class GameHandler : MonoBehaviour
{
    // Singletons
    public static EntityManager entityManager;
    public static GameHandler instance { get; set; }

    // Pathfinding
    public PathfindingGraph pathfindingGraph;

    // Mesh and materials
    public Mesh unitMesh;
    public Material unitMaterial;
    public GameObject markerPrefab;
    [SerializeField] private RectTransform selectionBox;

    // Input
    public float3 mouse_startPos;
    public float3 mouse_endPos;
    public bool showBox = false;

    private void Awake()
    {
        instance = this;
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        //selectionBox.gameObject.SetActive( false );

        pathfindingGraph = new PathfindingGraph();
        UnitSpawner unitSpawner = new UnitSpawner( unitMesh , unitMaterial );
    }
    private void Update()
    {
        /*if ( showBox )
            ShowBox();
        else
            selectionBox.gameObject.SetActive( false );*/
    }
    private void OnDisable()
    {
        if (pathfindingGraph != null)
            pathfindingGraph.Dispose();
    }

    private void ShowBox()
    {
        /*selectionBox.gameObject.SetActive( true );
        float3 squareStart = Camera.main.WorldToScreenPoint( mouse_startPos );
        mouse_endPos = Input.mousePosition;
        squareStart.z = 0f;

        float3 center = ( squareStart + mouse_endPos ) / 2;
        float sizeX = math.abs( squareStart.x - mouse_endPos.x );
        float sizeY = math.abs( squareStart.y - mouse_endPos.y );

        selectionBox.position = center;
        selectionBox.sizeDelta = new float2( sizeX , sizeY );*/

        showBox = false;
    }
}