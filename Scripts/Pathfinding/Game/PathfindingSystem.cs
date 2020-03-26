using System.Diagnostics;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

[AlwaysUpdateSystem]
public class PathfindingSystem : ComponentSystem
{
    private const int HEURISTIC_BIAS = 1;
    private const int MAX_JOBS_PER_FRAME = 8;
    private const int JOB_BATCH_SIZE = 2;

    private PathfindingGraph graph;

    private NativeList<Entity> jobEntities = new NativeList<Entity>( Allocator.Persistent );
    private NativeList<float2> jobStartPositions = new NativeList<float2>( Allocator.Persistent );
    private NativeList<float2> jobEndPositions = new NativeList<float2>( Allocator.Persistent );

    private Queue<Entity> entityQueue = new Queue<Entity>();
    private Queue<float2> startPositionQueue = new Queue<float2>();
    private Queue<float2> endPositionQueue = new Queue<float2>();

    private Stopwatch sw;
    private GameObject marker;

    protected override void OnStartRunning()
    {
        base.OnStartRunning();
        sw = new Stopwatch();
        graph = GameHandler.instance.pathfindingGraph;
        marker = GameHandler.instance.markerPrefab;
    }
    protected override void OnUpdate()
    {
        #region SETUP

        #region DEBUGGING

        sw.Reset();
        sw.Start();

        bool showTime = false;
        bool showMarkers = false;

        #endregion
        #region REAL STUFF

        jobEntities.Clear();
        jobStartPositions.Clear();
        jobEndPositions.Clear();

        // Get all entities with new pathfinding orders
        Entities.ForEach( ( Entity entity , ref PathFindingOrders pathFindingOrders , ref Translation translation ) =>
        {
            entityQueue.Enqueue( entity );
            startPositionQueue.Enqueue( new float2( translation.Value.x , translation.Value.z ) );
            endPositionQueue.Enqueue( pathFindingOrders.targetPosition );

            PostUpdateCommands.RemoveComponent<PathFindingOrders>( entity );
        } );

        #endregion

        #endregion
        #region JOB SCHEDULING      

        int numEntitiesWaiting = entityQueue.Count;
        if ( numEntitiesWaiting > 0 )
        {
            #region PRE PROCESSING

            int numJobs = MAX_JOBS_PER_FRAME;
            if ( numEntitiesWaiting < MAX_JOBS_PER_FRAME )
                numJobs = numEntitiesWaiting;

            showTime = true;

            jobEntities.Clear();
            jobStartPositions.Clear();
            jobEndPositions.Clear();

            for ( int i = 0; i < numJobs; i++ )
            {
                jobEntities.Add( entityQueue.Dequeue() );
                jobStartPositions.Add( startPositionQueue.Dequeue() );
                jobEndPositions.Add( endPositionQueue.Dequeue() );
            }

            #endregion
            #region JOB

            FindPathJob job = new FindPathJob
            {
                // GRID DATA
                neighbourOffsetArray = graph.neighbourOffsetArray ,
                heuristicBias = HEURISTIC_BIAS ,
                graphCellLength = graph.numCellsAcross ,
                graphCellSize = graph.cellSize ,
                graphClusterSize = graph.clusterSize ,
                graphNumClusters = graph.numClustersAcross ,
                // CLUSTER
                graphClusterEdgesLists = graph.clusterEdges ,
                graphClustersDense = graph.clustersDense ,
                graphIntraEdges = graph.intraEdges ,
                graphInterEdges = graph.interEdges ,
                // EDGE
                graphEdgeNeighbourList = graph.edgeNeighbourKeys ,
                graphEdgePathList = graph.pathKeysPerEdge ,
                graphEdgePaths = graph.edgePaths ,
                graphEdgePathPositions = graph.edgePathNodePositions ,
                graphEdgeNeighbours = graph.edgeNeighbours ,
                // NODE
                graphNodeToEdge = graph.nodeToEdge ,
                graphNodePositions = graph.nodePositions ,
                graphNodeWalkables = graph.nodeWalkables ,
                // ENTITY DATA
                entities = jobEntities.AsArray() ,
                startWorldPositions = jobStartPositions.AsArray() ,
                endWorldPositions = jobEndPositions.AsArray() ,
                // WRITABLE ENTITY DATA
                pathPositionBuffer = GetBufferFromEntity<PathPosition>() ,
                pathIndexComponentData = GetComponentDataFromEntity<PathIndex>()
            };
            JobHandle jobHandle = job.Schedule( numJobs , JOB_BATCH_SIZE );
            jobHandle.Complete();

            #endregion
            #region DEBUGGING

            sw.Stop();
            if ( showTime )
                UnityEngine.Debug.Log( sw.Elapsed );
            if ( showMarkers )
            {
                Entities.ForEach( ( Entity entity , DynamicBuffer<PathPosition> pathPositionBuffer ) =>
                {
                    for ( int i = 0; i < pathPositionBuffer.Length; i++ )
                    {
                        Vector3 position = new Vector3(
                            pathPositionBuffer[ i ].position.x ,
                            3 ,
                            pathPositionBuffer[ i ].position.y );

                        Object.Instantiate( marker , position , quaternion.identity );
                    }
                } );
            }

            #endregion
        }

        #endregion
    }
    protected override void OnDestroy()
    {
        jobEntities.Dispose();
        jobStartPositions.Dispose();
        jobEndPositions.Dispose();
        base.OnDestroy();
    }

    [BurstCompile] private struct FindPathJob : IJobParallelFor
    {
        // Grid Data
        [ReadOnly] public int heuristicBias;
        [ReadOnly] public int graphCellLength;
        [ReadOnly] public int graphNumClusters;
        [ReadOnly] public int graphCellSize;
        [ReadOnly] public int graphClusterSize;
        // CLUSTER
        [ReadOnly] public NativeArray<int2> graphClusterEdgesLists;
        [ReadOnly] public NativeArray<byte> graphClustersDense;
        [ReadOnly] public NativeArray<int> graphIntraEdges;
        [ReadOnly] public NativeArray<int> graphInterEdges;
        // EDGE
        [ReadOnly] public NativeArray<int2> graphEdgeNeighbourList; // NodeLength
        [ReadOnly] public NativeArray<int> graphEdgeNeighbours;
        [ReadOnly] public NativeArray<int2> graphEdgePathList; // NodeLength
        [ReadOnly] public NativeArray<int2> graphEdgePaths;
        [ReadOnly] public NativeArray<int2> graphEdgePathPositions;
        // NODE
        [ReadOnly] public NativeArray<int2> neighbourOffsetArray;
        [ReadOnly] public NativeArray<int> graphNodeToEdge; // NodeLength
        [ReadOnly] public NativeArray<int2> graphNodePositions; // NodeLength
        [ReadOnly] public NativeArray<byte> graphNodeWalkables; // NodeLength
        // Entity Data
        [ReadOnly] public NativeArray<float2> startWorldPositions;
        [ReadOnly] public NativeArray<float2> endWorldPositions;
        [ReadOnly] public NativeArray<Entity> entities;
        // Entity Data To Write To
        [NativeDisableContainerSafetyRestriction] public BufferFromEntity<PathPosition> pathPositionBuffer;
        [NativeDisableContainerSafetyRestriction] public ComponentDataFromEntity<PathIndex> pathIndexComponentData;

        public void Execute( int jobIndex )
        {
            #region SETUP

            // Get the current entity
            Entity entity = entities[ jobIndex ];
            pathPositionBuffer[ entity ].Clear();
            pathIndexComponentData[ entity ] = new PathIndex { index = -1 };

            // Get grid positions from world positions (float2 -> int2)
            float2 startGridPositionAsFloat = new float2( 
                startWorldPositions[ jobIndex ].x / graphCellSize , 
                startWorldPositions[ jobIndex ].y / graphCellSize );
            float2 endGridPositionAsFloat = new float2( 
                endWorldPositions[ jobIndex ].x / graphCellSize , 
                endWorldPositions[ jobIndex ].y / graphCellSize );
            int2 startGridPosition = new int2( 
                ( int ) math.round( startGridPositionAsFloat.x ) , 
                ( int ) math.round( startGridPositionAsFloat.y ) );
            int2 endGridPosition = new int2( 
                ( int ) math.round( endGridPositionAsFloat.x ) , 
                ( int ) math.round( endGridPositionAsFloat.y ) );

            // Get the cluster positions from the grid positions
            int2 startClusterPosition = new int2(
                startGridPosition.x / graphClusterSize ,
                startGridPosition.y / graphClusterSize );
            int2 endClusterPosition = new int2(
                endGridPosition.x / graphClusterSize ,
                endGridPosition.y / graphClusterSize );

            #endregion
            #region MEAT

            // If the start and end positions are in different clusters
            if ( !startClusterPosition.Equals( endClusterPosition ) )
            {                    
                // Get the start and end node indexes from their grid cluster positions
                int clusterIndex = startClusterPosition.x + startClusterPosition.y * graphNumClusters;
                int2 clusterEdgesKey = graphClusterEdgesLists[ clusterIndex ];
                // Check every edge in the cluster to see if we can reach it, and add it to the list if we can
                NativeList<int> startEdges = new NativeList<int>( Allocator.Temp );
                for ( int edgeIndex = clusterEdgesKey.x; edgeIndex < clusterEdgesKey.y; edgeIndex++ )
                    startEdges.Add( graphIntraEdges[ edgeIndex ] );

                // If there are no paths from start position to any of the edges in current cluster, break out
                if ( startEdges.Length != 0 )
                {
                    NativeList<int3> abstractPath =
                        FindAbstractPathInGraph( startEdges , startGridPosition , endGridPosition );

                    if ( abstractPath.Length > 0 )
                    {
                        NativeList<int2> concretePath = FindConcretePathFromAbstract( abstractPath );
                        // Write final path to entity buffer
                        float2 pathPosition = new float2( 0 , 0 );
                        for ( int i = 0; i < concretePath.Length; i++ )
                        {
                            pathPosition = new float2(
                                concretePath[ i ].x * graphCellSize ,
                                concretePath[ i ].y * graphCellSize );
                            pathPositionBuffer[ entity ].Add(
                                new PathPosition { position = pathPosition } );
                        }
                        pathIndexComponentData[ entity ] =
                            new PathIndex { index = pathPositionBuffer[ entity ].Length - 1 };

                        concretePath.Dispose();
                        abstractPath.Dispose();
                        startEdges.Dispose();
                        return;
                    }
                    else
                    {
                        pathIndexComponentData[ entity ] =
                            new PathIndex { index = -1 };
                        startEdges.Dispose();
                        abstractPath.Dispose();
                        return;
                    }
                }
                else
                {
                    pathIndexComponentData[ entity ] = 
                        new PathIndex { index = -1 };
                    startEdges.Dispose();
                    return;
                }
            }
            else // OTHERWISE, the start and end positions are in the same cluster, so just do a quick low-level a* search
            {
                NativeList<int2> path = 
                    FindLowLevelPathInCluster( startGridPosition , endGridPosition , startClusterPosition );

                if ( path.Length > 0 )
                {
                    float2 pathPosition = new float2( 0 , 0 );
                    for ( int i = 0; i < path.Length; i++ )
                    {
                        pathPosition = new float2(
                            path[ i ].x * graphCellSize ,
                            path[ i ].y * graphCellSize );
                        pathPositionBuffer[ entity ].Add( 
                            new PathPosition { position = pathPosition } );
                    }
                    pathIndexComponentData[ entity ] = 
                        new PathIndex { index = pathPositionBuffer[ entity ].Length - 1 };
                }
                else
                {
                    pathIndexComponentData[ entity ] = 
                        new PathIndex { index = -1 };
                }

                path.Dispose();
                return;
            }

            #endregion
        }

        private NativeList<int2> FindLowLevelPathInCluster( int2 startPosition , int2 endPosition , int2 clusterPosition )
        {
            #region DATA SETUP

            int PATH_NODE_ARRAY_SIZE = graphClusterSize * graphClusterSize;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( PATH_NODE_ARRAY_SIZE , -1 , PATH_NODE_ARRAY_SIZE + 1 );
            NativeArray<int> localIndexArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> graphIndexArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );

            // Initialize nodes from ReadOnly grid array
            for ( int localRow = 0; localRow < graphClusterSize; localRow++ )
            {
                for ( int localCol = 0; localCol < graphClusterSize; localCol++ )
                {
                    int localIndex = localCol + localRow * graphClusterSize;
                    int graphCol = localCol + graphClusterSize * clusterPosition.x;
                    int graphRow = localRow + graphClusterSize * clusterPosition.y;
                    int graphArrayIndex = graphCol + graphRow * graphCellLength;

                    localIndexArray[ localIndex ] = localIndex;
                    graphIndexArray[ localIndex ] = graphArrayIndex;
                    parentArray[ localIndex ] = -1;
                    gCostArray[ localIndex ] = int.MaxValue;
                    openArray[ localIndex ] = false;
                    closedArray[ localIndex ] = false;
                }
            }

            // Get and cache the start and end pathNodeIndices
            int endNodeX = endPosition.x - clusterPosition.x * graphClusterSize;
            int endNodeY = endPosition.y - clusterPosition.y * graphClusterSize;
            int startNodeX = startPosition.x - clusterPosition.x * graphClusterSize;
            int startNodeY = startPosition.y - clusterPosition.y * graphClusterSize;
            int endNodeIndex = endNodeX + endNodeY * graphClusterSize;
            int startNodeIndex = startNodeX + startNodeY * graphClusterSize;

            // Initialize the starting pathNode
            int hCost = ManhattenDistance( graphNodePositions[ graphIndexArray[ startNodeIndex ] ] , endPosition );
            gCostArray[ startNodeIndex ] = 0;
            hCostArray[ startNodeIndex ] = hCost;
            fCostArray[ startNodeIndex ] = hCost;
            openArray[ startNodeIndex ] = true;

            // Add the starting node to the open set
            openSet.Enqueue( startNodeIndex , hCost );

            #endregion
            #region SEARCH GRAPH

            while ( openSet.Length > 0 )
            {
                // Cache the pathNodeIndex we are working with during this iteration
                int currentNodeIndex =
                    openSet.DequeueMin( localIndexArray , closedArray );

                openArray[ currentNodeIndex ] = false;
                closedArray[ currentNodeIndex ] = true;

                // Break if we reached our goal
                if ( currentNodeIndex == endNodeIndex )
                    break;

                int2 currentNodePosition =
                    graphNodePositions[ graphIndexArray[ currentNodeIndex ] ];

                for ( int nIndex = 0; nIndex < neighbourOffsetArray.Length; nIndex++ )
                {
                    int2 neighbourPosition =
                        currentNodePosition + neighbourOffsetArray[ nIndex ];

                    if ( !ValidateGridPosition( neighbourPosition , clusterPosition , graphClusterSize ) )
                        continue;

                    int graphNeighbour =
                        neighbourPosition.x + neighbourPosition.y * graphCellLength;

                    if ( graphNodeWalkables[ graphNeighbour ] > 0 )
                        continue;

                    // Get the local neighbour index
                    int2 localNeighbourPosition = new int2(
                        neighbourPosition.x - clusterPosition.x * graphClusterSize ,
                        neighbourPosition.y - clusterPosition.y * graphClusterSize );
                    int localNeighbourIndex =
                        localNeighbourPosition.x + localNeighbourPosition.y * graphClusterSize;

                    // Skip if its closed (already searched)
                    if ( closedArray[ localNeighbourIndex ] )
                        continue;

                    // Calculate the cost to move from current node to neighbour node
                    int distanceCost =
                        ManhattenDistance( currentNodePosition , neighbourPosition );
                    int tentativeCost =
                        gCostArray[ currentNodeIndex ] + distanceCost;

                    if ( tentativeCost < gCostArray[ localNeighbourIndex ] )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition , endPosition );

                        parentArray[ localNeighbourIndex ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex ] = newHCost;
                        gCostArray[ localNeighbourIndex ] = tentativeCost;
                        fCostArray[ localNeighbourIndex ] = tentativeCost + hCost;

                        if ( !openArray[ localNeighbourIndex ] )
                        {
                            openArray[ localNeighbourIndex ] = true;
                            openSet.Enqueue( localNeighbourIndex , fCostArray[ localNeighbourIndex ] );
                        }
                    }
                }
            }

            #endregion
            #region TRACE PATH

            NativeList<int2> path = new NativeList<int2>( Allocator.Temp );
            NativeList<int2> smoothedPath = new NativeList<int2>( Allocator.Temp );
            int nodeIndex = endNodeIndex;

            if ( parentArray[ endNodeIndex ] == -1 )
            {
                openSet.Dispose();
                localIndexArray.Dispose();
                graphIndexArray.Dispose();
                parentArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();
                return path;
            }
            else
            {
                smoothedPath.Add( endPosition );
                path.Add( endPosition );

                while ( parentArray[ nodeIndex ] != -1 )
                {
                    path.Add( graphNodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ] );
                    nodeIndex = parentArray[ nodeIndex ];
                }
            }

            #endregion
            #region SMOOTH PATH

            if ( path.Length > 2 ) // If its less than or equal 2 theres no need to smooth the path
            {
                int fromIndex = 0;
                bool foundPath = false;

                while ( !foundPath )
                {
                    int currentIndex = fromIndex + 2; // Because the next index is always going to be in line of sight

                    if ( currentIndex > path.Length - 1 )
                        break;

                    while ( true )
                    {
                        int stopIndex = 
                            currentIndex - 1;
                        int graphNodeArrayIndex =
                            path[ stopIndex ].x + path[ stopIndex ].y * graphCellLength;

                        int2 start = path[ fromIndex ];
                        int2 end = path[ currentIndex ];

                        if ( !LOS_Node( start , end ) )
                        {
                            smoothedPath.Add( graphNodePositions[ graphNodeArrayIndex ] );
                            fromIndex = stopIndex;
                            break;
                        }
                        else
                        {
                            if ( currentIndex >= path.Length - 1 )
                            {
                                foundPath = true;
                                break;
                            }
                            currentIndex++;
                        }
                    }
                }
            }

            #endregion
            #region RETURN

            openSet.Dispose();
            localIndexArray.Dispose();
            graphIndexArray.Dispose();
            parentArray.Dispose();
            hCostArray.Dispose();
            gCostArray.Dispose();
            fCostArray.Dispose();
            openArray.Dispose();
            closedArray.Dispose();
            path.Dispose();

            return smoothedPath;

            #endregion
        }
        private NativeList<int3> FindAbstractPathInGraph( NativeList<int> startNodes , int2 startPosition , int2 endPosition )
        {
            #region DATA SETUP

            int EDGE_ARRAY_SIZE = graphIntraEdges.Length;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( EDGE_ARRAY_SIZE , -1 , EDGE_ARRAY_SIZE + 10 );
            NativeArray<int> graphNodeIntraIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> graphNodeInterIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentEdgeArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> parentPathIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( EDGE_ARRAY_SIZE , Allocator.Temp );

            void DisposeNativeContainers()
            {
                openSet.Dispose();
                graphNodeIntraIndexArray.Dispose();
                graphNodeInterIndexArray.Dispose();
                parentEdgeArray.Dispose();
                parentPathIndexArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();
            }

            // Initialize default array values
            for ( int i = 0; i < EDGE_ARRAY_SIZE; i++ )
            {
                graphNodeIntraIndexArray[ i ] = graphIntraEdges[ i ];
                graphNodeInterIndexArray[ i ] = graphInterEdges[ i ];
                parentEdgeArray[ i ] = -1;
                parentPathIndexArray[ i ] = -1;
                gCostArray[ i ] = int.MaxValue;
                openArray[ i ] = false;
                closedArray[ i ] = false;
            }

            // Initialize the starting nodes(edges) from the parameter
            for ( int i = 0; i < startNodes.Length; i++ )
            {
                int nodeIndex = startNodes[ i ];
                int edgeIndex = graphNodeToEdge[ nodeIndex ];

                hCostArray[ edgeIndex ] = ManhattenDistance( graphNodePositions[ nodeIndex ] , endPosition );
                gCostArray[ edgeIndex ] = ManhattenDistance( startPosition , graphNodePositions[ nodeIndex ] );
                fCostArray[ edgeIndex ] = gCostArray[ edgeIndex ] + hCostArray[ edgeIndex ];

                openSet.Enqueue( edgeIndex , fCostArray[ edgeIndex ] );
            }

            // So we can test when we have reached the end
            int2 endClusterPosition = new int2(
                endPosition.x / graphClusterSize ,
                endPosition.y / graphClusterSize );

            int endEdgeIndex = -1;

            #endregion
            #region FIND PATH

            while ( openSet.Length > 0 )
            {
                int currentEdge = openSet.DequeueMin( closedArray );
                int currentInterNode = graphNodeInterIndexArray[ currentEdge ];

                int2 clusterPosition = new int2(
                    graphNodePositions[ currentInterNode ].x / graphClusterSize ,
                    graphNodePositions[ currentInterNode ].y / graphClusterSize );

                if ( clusterPosition.Equals( endClusterPosition ) )
                {
                    endEdgeIndex = currentEdge;
                    break;
                }

                openArray[ currentEdge ] = false;
                closedArray[ currentEdge ] = true;

                int2 interNodeNeighbours =
                    graphEdgeNeighbourList[ currentInterNode ];
                for ( int i = interNodeNeighbours.x; i < interNodeNeighbours.y; i++ )
                {
                    int neighbourNode = graphEdgeNeighbours[ i ];
                    int neighbourEdge = graphNodeToEdge[ neighbourNode ];

                    if ( closedArray[ neighbourEdge ] )
                        continue;

                    int distanceCost = ManhattenDistance(
                        graphNodePositions[ currentInterNode ] , graphNodePositions[ neighbourNode ] );
                    int tentativeCost =
                        distanceCost + gCostArray[ currentEdge ];

                    if ( tentativeCost < gCostArray[ neighbourEdge ] )
                    {
                        int newHCost = ManhattenDistance( graphNodePositions[ neighbourNode ] , endPosition );
                        parentEdgeArray[ neighbourEdge ] = currentEdge;
                        parentPathIndexArray[ neighbourEdge ] = i;
                        hCostArray[ neighbourEdge ] = newHCost;
                        gCostArray[ neighbourEdge ] = tentativeCost;
                        fCostArray[ neighbourEdge ] = newHCost + tentativeCost;

                        if ( !openArray[ neighbourEdge ] )
                        {
                            openArray[ neighbourEdge ] = true;
                            openSet.Enqueue( neighbourEdge , fCostArray[ neighbourEdge ] );
                        }
                    }
                }
            }

            #endregion
            #region TRACE PATH

            NativeList<int3> pathPositions = new NativeList<int3>( Allocator.Temp );

            if ( endEdgeIndex != -1 )
            {
                // Add the end position and the inter-node it connects to
                int interNodeIndex =
                    graphNodeInterIndexArray[ endEdgeIndex ];
                pathPositions.Add( new int3(
                    endPosition.x ,
                    endPosition.y ,
                    0 ) );
                pathPositions.Add( new int3(
                    graphNodePositions[ interNodeIndex ].x ,
                    graphNodePositions[ interNodeIndex ].y ,
                    0 ) );

                // Trace the path
                int edgeIndex = endEdgeIndex;
                while ( parentEdgeArray[ edgeIndex ] != -1 )
                {
                    int3 pathNodeData = new int3(
                        graphNodePositions[ graphNodeIntraIndexArray[ edgeIndex ] ].x ,
                        graphNodePositions[ graphNodeIntraIndexArray[ edgeIndex ] ].y ,
                        parentPathIndexArray[ edgeIndex ] );
                    pathPositions.Add( pathNodeData );
                    edgeIndex = parentEdgeArray[ edgeIndex ];
                }

                // Add the internode of the first edge to the path and the starting position
                int interEdge =
                    graphNodeToEdge[ graphNodeInterIndexArray[ edgeIndex ] ];
                int interNode =
                    graphNodeInterIndexArray[ interEdge ];

                pathPositions.Add( new int3(
                    graphNodePositions[ interNode ].x ,
                    graphNodePositions[ interNode ].y ,
                    0 ) );
                pathPositions.Add( new int3(
                    startPosition.x ,
                    startPosition.y ,
                    0 ) );

                DisposeNativeContainers();
                return pathPositions;
            }
            else
            {
                DisposeNativeContainers();
                return pathPositions;
            }

            #endregion
        }
        private NativeList<int2> FindConcretePathFromAbstract( NativeList<int3> abstractPath )
        {
            NativeList<int2> concretePath = new NativeList<int2>( Allocator.Temp );

            #region START

            int fromIndex = 0; // endPosition
            int toIndex = 2; // endIntra, there is an inter before this

            int2 endNode = new int2(
                abstractPath[ 0 ].x ,
                abstractPath[ 0 ].y );
            int2 endInter = new int2(
                abstractPath[ 1 ].x ,
                abstractPath[ 1 ].y );
            int2 endCluster = new int2(
                endNode.x / graphClusterSize ,
                endNode.y / graphClusterSize );

            concretePath.Add( endNode );

            #endregion
            #region END CLUSTER PATH

            // Try to find low-level path in end cluster
            int endClusterIndex = endCluster.x + endCluster.y * graphClusterSize;

            if ( graphClustersDense[ endClusterIndex ] != 0 )
            {
                NativeList<int2> lowLevelPath = FindLowLevelPathInCluster( endInter , endNode , endCluster );

                if ( lowLevelPath.Length == 0 )
                {
                    lowLevelPath.Dispose();
                    concretePath.Dispose();
                    return new NativeList<int2>();
                }

                for ( int i = 0; i < lowLevelPath.Length; i++ )
                    concretePath.Add( lowLevelPath[ i ] );

                // if we find a path in the last cluster, we then start from the first index in the prevous cluster
                // this is endPos -> endInter -> endIntra = fromIndex # 2
                // we test to prev cluster which is endIntra -> prevIntra = toIndex = # 3
                fromIndex = 2;
                toIndex = 3;
            }
            else
            {
                toIndex++;
            }

            #endregion
            #region FILL ABSTRACT PATH

            // We want to stop testing clusters once we reach the first one. We need to be able to
            // find a low level path in the first cluster from the first intra-index to the start position
            // We also want to be able test test if clusters are clear from the fromIndex to the startPosition
            // if (start cluster is not dense, raycast from from to start)
            // else (findLowLevel path from toIndex - 1 to startPosition)
            
            // therefore, stopPoint is when toIndex == startPosition
            int lastIndex = abstractPath.Length - 1;

            while ( toIndex < lastIndex )
            {
                #region CHECK FROM CLUSTER
                // Check if our from cluster is dense
                // If it is, add the cached path and then move the from and start positions up (back) one
                int2 fromPosition = new int2(
                    abstractPath[ fromIndex ].x ,
                    abstractPath[ fromIndex ].y );
                int2 clusterPosition = new int2(
                    fromPosition.x / graphClusterSize ,
                    fromPosition.y / graphClusterSize );
                int clusterIndex =
                    clusterPosition.x + clusterPosition.y * graphClusterSize;

                if ( graphClustersDense[ clusterIndex ] != 0 )
                {
                    int2 cachedPathKey =
                        graphEdgePaths[ abstractPath[ fromIndex ].z ];

                    for ( int i = cachedPathKey.x; i < cachedPathKey.y; i++ )
                        concretePath.Add( graphEdgePathPositions[ i ] );

                    fromIndex++;
                    toIndex++;
                }

                #endregion
                #region RAYCAST CLUSTERS LOOP
                // Raycast until we are blocked or reach the end
                // If we get blocked, make the from position = toPosition - 1
                bool notBlockedOrAtEnd = true;

                while ( notBlockedOrAtEnd )
                {
                    int2 start = new int2(
                        abstractPath[ fromIndex ].x ,
                        abstractPath[ fromIndex ].y );
                    int2 end = new int2(
                        abstractPath[ toIndex ].x ,
                        abstractPath[ toIndex ].y );

                    if ( !LOS_Cluster( start , end ) )
                    {
                        int2 stopPosition = new int2(
                            abstractPath[ toIndex ].x ,
                            abstractPath[ toIndex ].y );

                        concretePath.Add( stopPosition );

                        fromIndex = toIndex;
                        toIndex = toIndex + 1;

                        notBlockedOrAtEnd = false;
                    }
                    else
                    {
                        toIndex++;
                        if (toIndex >= lastIndex)
                            notBlockedOrAtEnd = false;
                    }
                }

                #endregion
            }

            #endregion
            #region START CLUSTER PATH

            int2 toPosition = new int2(
                abstractPath[ toIndex - 1 ].x ,
                abstractPath[ toIndex - 1 ].y );
            int2 startPosition = new int2(
                abstractPath[ lastIndex ].x ,
                abstractPath[ lastIndex ].y );
            int2 firstClusterPosition = new int2(
                startPosition.x / graphClusterSize ,
                startPosition.y / graphClusterSize );
            int firstClusterIndex = 
                firstClusterPosition.x + firstClusterPosition.y * graphClusterSize;

            if ( graphClustersDense[ firstClusterIndex ] != 0 )
            {
                NativeList<int2> lowLevelPath = FindLowLevelPathInCluster( startPosition , toPosition , firstClusterPosition );

                for ( int i = 0; i < lowLevelPath.Length; i++ )
                    concretePath.Add( lowLevelPath[ i ] );

                lowLevelPath.Dispose();
            }
            else
            {
                int2 endPosition = new int2(
                    abstractPath[ lastIndex ].x ,
                    abstractPath[ lastIndex ].y );
                concretePath.Add( endPosition );
            }

            #endregion

            return concretePath;
        }

        private bool LOS_Cluster( int2 start , int2 end )
        {
            int2 dif = new int2(
                math.abs( end.x - start.x ) ,
                math.abs( end.y - start.y ) );
            int2 pos = new int2(
                start.x ,
                start.y );
            int2 inc = new int2(
                ( end.x > start.x ) ? 1 : -1 ,
                ( end.y > start.y ) ? 1 : -1 );

            int numIterations = 1 + dif.x + dif.y;
            int error = dif.x - dif.y;
            int walkable = 0;

            dif.x *= 2;
            dif.y *= 2;

            for ( ; numIterations > 0; --numIterations )
            {
                int2 clusterPosition = new int2(
                    pos.x / graphClusterSize ,
                    pos.y / graphClusterSize );
                int clusterIndex = clusterPosition.x + clusterPosition.y * graphClusterSize;

                walkable += graphClustersDense[ clusterIndex ];

                if ( error > 0 ) // more steps in x
                {
                    pos.x += inc.x;
                    error -= dif.y;
                }
                else // more steps in y
                {
                    pos.y += inc.y;
                    error += dif.x;
                }
            }

            return walkable == 0;
        }
        private bool LOS_Node( int2 start , int2 end )
        {
            int2 dif = new int2(
                math.abs( end.x - start.x ) ,
                math.abs( end.y - start.y ) );
            int2 pos = new int2(
                start.x ,
                start.y );
            int2 inc = new int2(
                ( end.x > start.x ) ? 1 : -1 ,
                ( end.y > start.y ) ? 1 : -1 );

            int numIterations = 1 + dif.x + dif.y;
            int error = dif.x - dif.y;
            int walkable = 0;

            dif.x *= 2;
            dif.y *= 2;

            for ( ; numIterations > 0; --numIterations )
            {
                walkable += graphNodeWalkables[ pos.x + pos.y * graphCellLength ];

                if ( error > 0 ) // more steps in x
                {
                    pos.x += inc.x;
                    error -= dif.y;
                }
                else // more steps in y
                {
                    pos.y += inc.y;
                    error += dif.x;
                }
            }

            return walkable == 0;
        }

        private short ManhattenDistance( int2 positionA , int2 positionB )
        {
            int2 distance = positionB - positionA;
            return ( short ) ( math.abs( distance.x ) + math.abs( distance.y ) );
        }
        private bool ValidateGridPosition( int2 position , int2 clusterPosition , int2 clusterSize )
        {
            int2 min = new int2(
                clusterPosition.x * graphClusterSize ,
                clusterPosition.y * graphClusterSize );

            return
                position.x >= min.x && position.x < min.x + clusterSize.x &&
                position.y >= min.y && position.y < min.y + clusterSize.y;
        }
    }
}