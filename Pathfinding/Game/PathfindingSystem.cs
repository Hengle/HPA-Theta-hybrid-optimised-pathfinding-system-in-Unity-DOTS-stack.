using System.Diagnostics;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using Formation;

[UpdateInGroup( typeof( SimulationSystemGroup ) )]
[UpdateBefore( typeof( UnitHeadingSystem ) )]
public class PathfindingSystem : SystemBase
{
    private const int HEURISTIC_BIAS = 1;
    private const int MAX_JOBS_PER_FRAME = 8;
    private const int JOB_BATCH_SIZE = 2;

    private PathfindingGraph graph;

    private NativeList<Entity> jobEntities = new NativeList<Entity>( Allocator.Persistent );
    private NativeList<float2> jobStartPositions = new NativeList<float2>( Allocator.Persistent );
    private NativeList<float2> jobEndPositions = new NativeList<float2>( Allocator.Persistent );

    private NativeQueue<Entity> entityQueue = new NativeQueue<Entity>( Allocator.Persistent );
    private NativeQueue<float2> startPositionQueue = new NativeQueue<float2>( Allocator.Persistent );
    private NativeQueue<float2> endPositionQueue = new NativeQueue<float2>( Allocator.Persistent );

    private JobHandle jobHandle;
    private EntityQuery entityQuery;

    private Stopwatch sw;
    private GameObject marker;
    private bool showTime = false;
    private bool showMarkers = false;

    protected override void OnStartRunning()
    {
        base.OnStartRunning();
        sw = new Stopwatch();
        graph = GameHandler.instance.pathfindingGraph;
        marker = GameHandler.instance.markerPrefab;
    }
    protected override void OnUpdate()
    {
        bool jobRan = false;

        SetupDebugging();

        entityQuery = GetEntityQuery( typeof( FormationPathFindingState ) );

        CollectEntitiesWithNewTargetJob collectJob = new CollectEntitiesWithNewTargetJob
        {
            entityQueue = entityQueue ,
            startPositionQueue = startPositionQueue ,
            endPositionQueue = endPositionQueue ,
            archetypeFormationPosition = GetArchetypeChunkComponentType<FormationPosition>() ,
            archetypePathfindingState = GetArchetypeChunkComponentType<FormationPathFindingState>() ,
            archetypeTargetPosition = GetArchetypeChunkComponentType<FormationTargetPosition>() ,
            entityType = GetArchetypeChunkEntityType() ,
        };
        collectJob.Run( entityQuery );

        if ( entityQueue.Count > 0 )
        {
            jobRan = true;
            int numJobs = FillEntityLists();

            FindPathJob job = new FindPathJob
            {
                // GRID DATA
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
                graphNodeNeighbourKeys = graph.nodeNeighbourKeys ,
                graphNodeNeighbours = graph.nodeNeighbours ,
                graphNodeToEdge = graph.nodeToEdge ,
                graphNodePositions = graph.nodePositions ,
                graphNodeWalkables = graph.nodeWalkables ,
                // READ ENTITY DATA
                entities = jobEntities.AsArray() ,
                startWorldPositions = jobStartPositions.AsArray() ,
                endWorldPositions = jobEndPositions.AsArray() ,
                targetRotationComponentData = GetComponentDataFromEntity<FormationTargetRotation>( true ) ,
                // WRITE ENTITY DATA
                pathfindingStateComponentData = GetComponentDataFromEntity<FormationPathFindingState>( false ) ,
                pathIndexComponentData = GetComponentDataFromEntity<PathIndex>( false ) ,
                pathPositionBufferEntity = GetBufferFromEntity<PathPosition>( false ) ,
            };
            jobHandle = job.Schedule( numJobs , JOB_BATCH_SIZE , Dependency );
            Dependency = jobHandle;

            #region DEBUGGING

            sw.Stop();

            if ( showTime && jobRan )
                UnityEngine.Debug.Log( sw.Elapsed );
            if ( showMarkers )
                DebugDrawMarkers();

            #endregion
        }
    }
    protected override void OnDestroy()
    {
        entityQueue.Dispose();
        startPositionQueue.Dispose();
        endPositionQueue.Dispose();
        jobEntities.Dispose();
        jobStartPositions.Dispose();
        jobEndPositions.Dispose();
        base.OnDestroy();
    }

    private void SetupDebugging()
    {
        sw.Reset();
        sw.Start();
    }
    private void DebugDrawMarkers()
    {
        Entities.WithoutBurst().ForEach( ( Entity entity , DynamicBuffer<PathPosition> pathPositionBuffer ) =>
        {
            for ( int i = 0; i < pathPositionBuffer.Length; i++ )
            {
                Vector3 position = new Vector3(
                    pathPositionBuffer[ i ].Value.x ,
                    3 ,
                    pathPositionBuffer[ i ].Value.y );

                Object.Instantiate( marker , position , quaternion.identity );
            }
        } ).Run();
    }
    private int FillEntityLists()
    {
        int numJobs = MAX_JOBS_PER_FRAME;
        if ( entityQueue.Count < MAX_JOBS_PER_FRAME )
            numJobs = entityQueue.Count;

        jobEntities.Clear();
        jobStartPositions.Clear();
        jobEndPositions.Clear();

        for ( int i = 0; i < numJobs; i++ )
        {
            jobEntities.Add( entityQueue.Dequeue() );
            jobStartPositions.Add( startPositionQueue.Dequeue() );
            jobEndPositions.Add( endPositionQueue.Dequeue() );
        }

        return numJobs;
    }

    [BurstCompile] private struct CollectEntitiesWithNewTargetJob : IJobChunk
    {
        #region FIELDS

        [ReadOnly] public ArchetypeChunkComponentType<FormationPosition> archetypeFormationPosition;
        [ReadOnly] public ArchetypeChunkComponentType<FormationTargetPosition> archetypeTargetPosition;
        [ReadOnly] public ArchetypeChunkEntityType entityType;

        [WriteOnly] public NativeQueue<Entity> entityQueue;
        [WriteOnly] public NativeQueue<float2> startPositionQueue;
        [WriteOnly] public NativeQueue<float2> endPositionQueue;
        public ArchetypeChunkComponentType<FormationPathFindingState> archetypePathfindingState;

        #endregion
        public void Execute( ArchetypeChunk chunk , int chunkIndex , int firstEntityIndex )
        {
            NativeArray<Entity> chunkEntity = chunk.GetNativeArray( entityType );
            NativeArray<FormationPathFindingState> chunkPathfindingState = chunk.GetNativeArray( archetypePathfindingState );
            NativeArray<FormationTargetPosition> chunkTargetPosition = chunk.GetNativeArray( archetypeTargetPosition );
            NativeArray<FormationPosition> chunkFormationPosition = chunk.GetNativeArray( archetypeFormationPosition );

            for ( int i = 0; i < chunk.Count; i++ )
            {
                if ( chunkPathfindingState[ i ].Value == PathfindingState.hasNewTarget )
                {
                    entityQueue.Enqueue( chunkEntity[ i ] );
                    startPositionQueue.Enqueue( new float2( chunkFormationPosition[ i ].x , chunkFormationPosition[ i ].y ) );
                    endPositionQueue.Enqueue( new float2( chunkTargetPosition[ i ].x , chunkTargetPosition[ i ].y ) );

                    chunkPathfindingState[ i ] = new FormationPathFindingState { Value = PathfindingState.findingPath };
                }
            }
        }
    }
    [BurstCompile] private unsafe struct FindPathJob : IJobParallelFor
    {
        #region FIELDS

        // Grid Data
        [ReadOnly] public int heuristicBias;
        [ReadOnly] public int graphCellLength;
        [ReadOnly] public int graphNumClusters;
        [ReadOnly] public int graphCellSize;
        [ReadOnly] public int graphClusterSize;
        // CLUSTER
        [ReadOnly] public NativeArray<int2>      graphClusterEdgesLists;
        [ReadOnly] public NativeArray<byte>      graphClustersDense;
        [ReadOnly] public NativeArray<int>       graphIntraEdges;
        [ReadOnly] public NativeArray<int>       graphInterEdges;
        // EDGE
        [ReadOnly] public NativeArray<int2>      graphEdgeNeighbourList; // NodeLength
        [ReadOnly] public NativeArray<int>       graphEdgeNeighbours;
        [ReadOnly] public NativeArray<int2>      graphEdgePathList; // NodeLength
        [ReadOnly] public NativeArray<int2>      graphEdgePaths;
        [ReadOnly] public NativeArray<int2>      graphEdgePathPositions;
        // NODE
        [ReadOnly] public NativeArray<int2>      graphNodeNeighbourKeys;
        [ReadOnly] public NativeArray<int>       graphNodeNeighbours;
        [ReadOnly] public NativeArray<int>       graphNodeToEdge; // NodeLength
        [ReadOnly] public NativeArray<int2>      graphNodePositions; // NodeLength
        [ReadOnly] public NativeArray<byte>      graphNodeWalkables; // NodeLength
        // Entity Data
        [ReadOnly] public NativeArray<float2>    startWorldPositions;
        [ReadOnly] public NativeArray<float2>    endWorldPositions;
        [ReadOnly] public NativeArray<Entity>    entities;
        [ReadOnly] public ComponentDataFromEntity<FormationTargetRotation> targetRotationComponentData;
        // Entity Data To Write To
        [NativeDisableContainerSafetyRestriction] public ComponentDataFromEntity<FormationPathFindingState> pathfindingStateComponentData;
        [NativeDisableContainerSafetyRestriction] public ComponentDataFromEntity<PathIndex> pathIndexComponentData;
        [NativeDisableContainerSafetyRestriction] public BufferFromEntity<PathPosition>     pathPositionBufferEntity;

        private struct AbstractNode
        {
            public int2 position;
            public int path;
        }

        #endregion
        public void Execute( int jobIndex )
        {
            Entity entity = entities[ jobIndex ];
            float2 startGridPositionAsFloat = startWorldPositions[ jobIndex ] / graphCellSize;
            float2 endGridPositionAsFloat = endWorldPositions[ jobIndex ] / graphCellSize;
            int2 startGridPosition = ( int2 ) math.round( startGridPositionAsFloat );
            int2 endGridPosition = ( int2 ) math.round( endGridPositionAsFloat );
            int2 startClusterPosition = startGridPosition / graphClusterSize;
            int2 endClusterPosition = endGridPosition / graphClusterSize;

            pathfindingStateComponentData[ entity ] = new FormationPathFindingState { Value = PathfindingState.free };

            // If the start and end positions are in different clusters
            if ( !startClusterPosition.Equals( endClusterPosition ) )
            {
                int clusterIndex = startClusterPosition.x + startClusterPosition.y * graphNumClusters;
                int2 clusterEdgesKey = graphClusterEdgesLists[ clusterIndex ];

                // If there are no paths from start position to any of the edges in current cluster, break out
                if ( clusterEdgesKey.y - clusterEdgesKey.x > 0 )
                {
                    NativeList<AbstractNode> abstractPath = FindAbstractPathInGraph( clusterEdgesKey , startGridPosition , endGridPosition );
                    if ( abstractPath.Length > 0 )
                    {
                        pathPositionBufferEntity[ entity ].Clear();
                        pathPositionBufferEntity[ entity ].Add( new PathPosition { Value = endWorldPositions[ jobIndex ] } );
                        NativeList<int2> concretePath = FindConcretePathFromAbstract( abstractPath );
                        for ( int i = 1; i < concretePath.Length; i++ )
                        {
                            float2 pathPosition = concretePath[ i ] * graphCellSize;
                            pathPositionBufferEntity[ entity ].Add( new PathPosition { Value = pathPosition } );
                        }
                        pathIndexComponentData[ entity ] = new PathIndex { Value = pathPositionBufferEntity[ entity ].Length - 1 };
                        concretePath.Dispose();
                        abstractPath.Dispose();
                        return;
                    }
                    abstractPath.Dispose();
                }
            }
            else // OTHERWISE, the start and end positions are in the same cluster, so just do a quick low-level a* search
            {
                FixedList4096<int2> path = FindLowLevelPathInCluster( startGridPosition , endGridPosition , startClusterPosition );
                if ( path.Length > 0 )
                {
                    pathPositionBufferEntity[ entity ].Clear();
                    pathPositionBufferEntity[ entity ].Add( new PathPosition { Value = endWorldPositions[ jobIndex ] } );
                    for ( int i = 1; i < path.Length; i++ )
                    {
                        float2 pathPosition = path[ i ] * graphCellSize;
                        pathPositionBufferEntity[ entity ].Add( new PathPosition { Value = pathPosition } );
                    }
                    pathIndexComponentData[ entity ] = new PathIndex { Value = pathPositionBufferEntity[ entity ].Length - 1 };
                    return;
                }
            }

            // If no path was found we will reach here
            return;
        }
        #region METHODS
        private FixedList4096<int2> FindLowLevelPathInCluster( int2 startPosition , int2 endPosition , int2 clusterPosition )
        {
            #region DATA SETUP

            int PATH_NODE_ARRAY_SIZE = graphClusterSize * graphClusterSize;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( PATH_NODE_ARRAY_SIZE , -1 , PATH_NODE_ARRAY_SIZE + 1 );
            NativeArray<int> graphIndexArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> parentArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> hCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> fCostArray = new NativeArray<int>( PATH_NODE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<bool> openArray = new NativeArray<bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.ClearMemory );
            NativeArray<bool> closedArray = new NativeArray<bool>( PATH_NODE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.ClearMemory );

            int intValue = -1;
            void* pointer = ( void* ) &intValue;
            UnsafeUtility.MemCpyReplicate( parentArray.GetUnsafePtr() , pointer , sizeof( int ) , PATH_NODE_ARRAY_SIZE );
            intValue = int.MaxValue;
            UnsafeUtility.MemCpyReplicate( gCostArray.GetUnsafePtr() , pointer , sizeof( int ) , PATH_NODE_ARRAY_SIZE );

            /*for ( int localRow = 0; localRow < graphClusterSize; localRow++ )
            {
                for ( int localCol = 0; localCol < graphClusterSize; localCol++ )
                {
                    int2 localPos = new int2( localCol , localRow );
                    int2 graphPos = localPos + graphClusterPos;

                    int localIndex = localCol + localRow * graphClusterSize;
                    int graphArrayIndex = graphPos.x + graphPos.y * graphCellLength;

                    graphIndexArray[ localIndex ] = graphArrayIndex;
                }
            }*/
            int2 graphClusterPos = graphClusterSize * clusterPosition;
            int4 graphClusterX = new int4( clusterPosition.x * graphClusterSize );
            int4 graphClusterY = new int4( clusterPosition.y * graphClusterSize );
            int4 indexOffest = new int4( 0 , 1 , 2 , 3 );
            int vectorLoopLength = graphClusterSize - 4;
            for ( int localRow = 0; localRow < graphClusterSize; localRow++ )
            {
                int localCol = 0;

                for ( ; localCol < vectorLoopLength; localCol += 4 )
                {
                    int localIndex = localCol + localRow * graphClusterSize;
                    int4 localCol4 = new int4( localCol );
                    int4 localRow4 = new int4( localRow );
                    int4 graphX = localCol4 + indexOffest + graphClusterX;
                    int4 graphY = localRow4 + graphClusterY;
                    int4 graphArrayIndex = graphX + graphY * graphCellLength;
                    graphIndexArray.ReinterpretStore<int4>( localIndex , graphArrayIndex );
                }
                for ( ; localCol < graphClusterSize; localCol++ )
                {
                    int2 localPos = new int2( localCol , localRow );
                    int2 graphPos = localPos + graphClusterPos;

                    int localIndex = localCol + localRow * graphClusterSize;
                    int graphArrayIndex = graphPos.x + graphPos.y * graphCellLength;

                    graphIndexArray[ localIndex ] = graphArrayIndex;
                }
            }

            // Get and cache the start and end pathNodeIndices
            int2 clusterNodePos = clusterPosition * graphClusterSize;
            int2 endNodePos = endPosition - clusterNodePos;
            int2 startNodePos = startPosition - clusterNodePos;
            int endNodeIndex = endNodePos.x + endNodePos.y * graphClusterSize;
            int startNodeIndex = startNodePos.x + startNodePos.y * graphClusterSize;

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
                int currentNodeIndex = openSet.DequeueMin( closedArray );

                openArray[ currentNodeIndex ] = false;
                closedArray[ currentNodeIndex ] = true;

                // Break if we reached our goal
                if ( currentNodeIndex == endNodeIndex )
                    break;

                int2 currentNodePosition = graphNodePositions[ graphIndexArray[ currentNodeIndex ] ];
                int2 neighbourKey = graphNodeNeighbourKeys[ graphIndexArray[ currentNodeIndex ] ];
                int i = neighbourKey.x;
                int vectorLength = neighbourKey.y - 4;

                /*for ( ; i < neighbourKey.y; i++ )
                {
                    int2 neighbourPosition = graphNodePositions[ graphNodeNeighbours[ i ] ];
                    int2 localNeighbourPosition = neighbourPosition - clusterPosition * graphClusterSize;
                    int localNeighbourIndex = localNeighbourPosition.x + localNeighbourPosition.y * graphClusterSize;

                    // Skip if its closed (already searched)
                    if ( closedArray[ localNeighbourIndex ] )
                        continue;

                    // Calculate the cost to move from current node to neighbour node
                    int distanceCost = ManhattenDistance( currentNodePosition , neighbourPosition );
                    int tentativeCost = gCostArray[ currentNodeIndex ] + distanceCost;

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
                }*/

                /*int4 clusterXScaled = new int4( clusterPosition.x * graphClusterSize );
                int4 clusterYScaled = new int4( clusterPosition.y * graphClusterSize );

                for ( ; i < vectorLength; i += 4 )
                {
                    int2x4 neighbourPosition = graphNodePositions.ReinterpretLoad<int2x4>( graphNodeNeighbours[ i ] );
                    int4 neighbourX = new int4( neighbourPosition.c0.x , neighbourPosition.c1.x , neighbourPosition.c2.x , neighbourPosition.c3.x );
                    int4 neighbourY = new int4( neighbourPosition.c0.y , neighbourPosition.c1.y , neighbourPosition.c2.y , neighbourPosition.c3.y );

                    int4 localNeighbourX = neighbourX - clusterXScaled;
                    int4 localNeighbourY = neighbourY - clusterYScaled;
                    int4 localNeighbourIndex = localNeighbourX + localNeighbourY * graphClusterSize;

                    int4 closedArrayBranch = math.select( new int4( 1 ) , new int4( 0 ) , closedArray.ReinterpretLoad<bool4>( localNeighbourIndex.x ) );

                    int4 differenceX = neighbourX - new int4( currentNodePosition.x );
                    int4 differenceY = neighbourY - new int4( currentNodePosition.y );

                    int4 distanceCost = math.abs( differenceX ) + math.abs( differenceY );
                    int4 tentativeCost = new int4( gCostArray[ currentNodeIndex ] + distanceCost );

                    int4 gcost = gCostArray.ReinterpretLoad<int4>( localNeighbourIndex.x ) * closedArrayBranch;
                    bool4 lesser = tentativeCost < gcost;

                    if ( lesser.x )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition.c0 , endPosition );

                        parentArray[ localNeighbourIndex.x ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex.x ] = newHCost;
                        gCostArray[ localNeighbourIndex.x ] = tentativeCost.x;
                        fCostArray[ localNeighbourIndex.x ] = tentativeCost.x + hCost;

                        if ( !openArray[ localNeighbourIndex.x ] )
                        {
                            openArray[ localNeighbourIndex.x ] = true;
                            openSet.Enqueue( localNeighbourIndex.x , fCostArray[ localNeighbourIndex.x ] );
                        }
                    }
                    if ( lesser.y )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition.c1 , endPosition );

                        parentArray[ localNeighbourIndex.y ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex.y ] = newHCost;
                        gCostArray[ localNeighbourIndex.y ] = tentativeCost.y;
                        fCostArray[ localNeighbourIndex.y ] = tentativeCost.y + hCost;

                        if ( !openArray[ localNeighbourIndex.y ] )
                        {
                            openArray[ localNeighbourIndex.y ] = true;
                            openSet.Enqueue( localNeighbourIndex.y , fCostArray[ localNeighbourIndex.y ] );
                        }
                    }
                    if ( lesser.z )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition.c2 , endPosition );

                        parentArray[ localNeighbourIndex.z ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex.z ] = newHCost;
                        gCostArray[ localNeighbourIndex.z ] = tentativeCost.z;
                        fCostArray[ localNeighbourIndex.z ] = tentativeCost.z + hCost;

                        if ( !openArray[ localNeighbourIndex.z ] )
                        {
                            openArray[ localNeighbourIndex.z ] = true;
                            openSet.Enqueue( localNeighbourIndex.z , fCostArray[ localNeighbourIndex.z ] );
                        }
                    }
                    if ( lesser.w )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition.c3 , endPosition );

                        parentArray[ localNeighbourIndex.w ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex.w ] = newHCost;
                        gCostArray[ localNeighbourIndex.w ] = tentativeCost.w;
                        fCostArray[ localNeighbourIndex.w ] = tentativeCost.w + hCost;

                        if ( !openArray[ localNeighbourIndex.w ] )
                        {
                            openArray[ localNeighbourIndex.w ] = true;
                            openSet.Enqueue( localNeighbourIndex.w , fCostArray[ localNeighbourIndex.w ] );
                        }
                    }
                }*/

                for ( ; i < neighbourKey.y; i++ )
                {
                    int2 neighbourPosition = graphNodePositions[ graphNodeNeighbours[ i ] ];
                    int2 localNeighbourPosition = neighbourPosition - clusterPosition * graphClusterSize;
                    int localNeighbourIndex = localNeighbourPosition.x + localNeighbourPosition.y * graphClusterSize;

                    // Skip if its closed (already searched)
                    if ( closedArray[ localNeighbourIndex ] )
                        continue;

                    // Calculate the cost to move from current node to neighbour node
                    int distanceCost = ManhattenDistance( currentNodePosition , neighbourPosition );
                    int tentativeCost = gCostArray[ currentNodeIndex ] + distanceCost;

                    if ( tentativeCost < gCostArray[ localNeighbourIndex ] )
                    {
                        int newHCost = ManhattenDistance( neighbourPosition , endPosition );

                        parentArray[ localNeighbourIndex ] = currentNodeIndex;
                        hCostArray[ localNeighbourIndex ] = newHCost;
                        gCostArray[ localNeighbourIndex ] = tentativeCost;
                        fCostArray[ localNeighbourIndex ] = tentativeCost + newHCost;

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
            FixedList4096<int2> smoothedPath = new FixedList4096<int2>(); //Allocator.Temp );

            int nodeIndex = endNodeIndex;

            if ( parentArray[ endNodeIndex ] == -1 )
            {
                openSet.Dispose();
                graphIndexArray.Dispose();
                parentArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();
                return smoothedPath;
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
                int currentIndex = fromIndex + 2;
                int stopIndex = path.Length - 1;

                while ( currentIndex <= stopIndex )
                {
                    int2 start = path[ fromIndex ];
                    int2 end = path[ currentIndex ];

                    if ( !LOS_Node( start , end ) )
                    {
                        int nextFromIndex = currentIndex - 1;
                        int graphNodeArrayIndex = path[ nextFromIndex ].x + path[ nextFromIndex ].y * graphCellLength;

                        smoothedPath.Add( graphNodePositions[ graphNodeArrayIndex ] );
                        fromIndex = nextFromIndex;
                    }

                    currentIndex++;
                }
            }

            #endregion
            #region RETURN

            openSet.Dispose();
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
        private NativeList<AbstractNode> FindAbstractPathInGraph( int2 startEdgesKey , int2 startPosition , int2 endPosition )
        {
            #region DATA SETUP

            int EDGE_ARRAY_SIZE = graphIntraEdges.Length;

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( EDGE_ARRAY_SIZE , -1 , EDGE_ARRAY_SIZE + 10 );
            NativeArray<int> parentEdgeArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> parentPathIndexArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> hCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.UninitializedMemory );
            NativeArray<int> fCostArray = new NativeArray<int>( EDGE_ARRAY_SIZE , Allocator.Temp );
            NativeArray<bool> openArray = new NativeArray<bool>( EDGE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.ClearMemory );
            NativeArray<bool> closedArray = new NativeArray<bool>( EDGE_ARRAY_SIZE , Allocator.Temp , NativeArrayOptions.ClearMemory );

            int intValue = -1;
            void* pointer = ( void* ) &intValue;
            UnsafeUtility.MemCpyReplicate( parentEdgeArray.GetUnsafePtr() , pointer , sizeof( int ) , EDGE_ARRAY_SIZE );
            UnsafeUtility.MemCpyReplicate( parentPathIndexArray.GetUnsafePtr() , pointer , sizeof( int ) , EDGE_ARRAY_SIZE );
            intValue = int.MaxValue;
            UnsafeUtility.MemCpyReplicate( gCostArray.GetUnsafePtr() , pointer , sizeof( int ) , EDGE_ARRAY_SIZE );

            // Initialize the starting nodes(edges) from the parameter
            for ( int i = startEdgesKey.x; i < startEdgesKey.y; i++ )
            {
                int nodeIndex = graphIntraEdges[ i ];
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
                int currentInterNode = graphInterEdges[ currentEdge ]; // graphNodeInterIndexArray[ currentEdge ];

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

                int2 interNodeNeighbours = graphEdgeNeighbourList[ currentInterNode ];
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

            NativeList<AbstractNode> pathNodes = new NativeList<AbstractNode>( Allocator.Temp );

            if ( endEdgeIndex != -1 )
            {
                // Add the end position and the inter-node it connects to
                int interNodeIndex = graphInterEdges[ endEdgeIndex ];
                pathNodes.Add( new AbstractNode
                {
                    position = endPosition ,
                    path = 0
                } );
                pathNodes.Add( new AbstractNode
                {
                    position = graphNodePositions[ interNodeIndex ] ,
                    path = 0
                } );

                // Trace the path
                int edgeIndex = endEdgeIndex;
                while ( parentEdgeArray[ edgeIndex ] != -1 )
                {
                    AbstractNode node = new AbstractNode();
                    node.position = graphNodePositions[ graphIntraEdges[ edgeIndex ] ];
                    node.path = parentPathIndexArray[ edgeIndex ];

                    pathNodes.Add( node );
                    edgeIndex = parentEdgeArray[ edgeIndex ];
                }

                // Add the internode of the first edge to the path and the starting position
                int interEdge = graphNodeToEdge[ graphInterEdges[ edgeIndex ] ];
                int interNode = graphInterEdges[ interEdge ];

                pathNodes.Add( new AbstractNode
                {
                    position = graphNodePositions[ interNode ] ,
                    path = 0
                } );
                pathNodes.Add( new AbstractNode
                {
                    position = startPosition ,
                    path = 0
                } );

                openSet.Dispose();
                parentEdgeArray.Dispose();
                parentPathIndexArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();

                return pathNodes;
            }
            else
            {
                openSet.Dispose();
                parentEdgeArray.Dispose();
                parentPathIndexArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();

                return pathNodes;
            }

            #endregion
        }
        private NativeList<int2> FindConcretePathFromAbstract( NativeList<AbstractNode> abstractPath )
        {
            NativeList<int2> concretePath = new NativeList<int2>( Allocator.Temp );

            #region START

            int fromIndex = 0; // endPosition
            int toIndex = 2; // endIntra, there is an inter before this

            int2 endNode = abstractPath[ 0 ].position;
            int2 endInter = abstractPath[ 1 ].position;
            int2 endCluster = endNode / graphClusterSize;

            concretePath.Add( endNode );

            #endregion
            #region END CLUSTER PATH

            // Try to find low-level path in end cluster
            int endClusterIndex = endCluster.x + endCluster.y * graphClusterSize;

            if ( graphClustersDense[ endClusterIndex ] != 0 )
            {
                FixedList4096<int2> lowLevelPath = FindLowLevelPathInCluster( endInter , endNode , endCluster );

                if ( lowLevelPath.Length == 0 )
                {
                    //lowLevelPath.Dispose();
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
                //toIndex++;
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
                int2 fromPosition = abstractPath[ fromIndex ].position;
                int2 clusterPosition = fromPosition / graphClusterSize;
                int clusterIndex = clusterPosition.x + clusterPosition.y * graphClusterSize;

                if ( graphClustersDense[ clusterIndex ] != 0 )
                {
                    int2 cachedPathKey = graphEdgePaths[ abstractPath[ fromIndex ].path ];

                    for ( int i = cachedPathKey.x; i < cachedPathKey.y; i++ )
                        concretePath.Add( graphEdgePathPositions[ i ] );

                    //fromIndex++;
                    //toIndex++;
                }

                #endregion
                #region RAYCAST CLUSTERS LOOP
                // Raycast until we are blocked or reach the end
                // If we get blocked, make the from position = toPosition - 1
                bool notBlockedOrAtEnd = true;

                while ( notBlockedOrAtEnd )
                {
                    int2 start = abstractPath[ fromIndex ].position;
                    int2 end = abstractPath[ toIndex ].position;

                    if ( !LOS_Cluster( start , end ) )
                    {
                        concretePath.Add( abstractPath[ toIndex ].position );
                        fromIndex = toIndex;
                        toIndex++;
                        notBlockedOrAtEnd = false;
                    }
                    else
                    {
                        toIndex++;
                        if ( toIndex >= lastIndex )
                            notBlockedOrAtEnd = false;
                    }
                }

                #endregion
            }

            #endregion
            #region START CLUSTER PATH

            int2 toPosition = abstractPath[ toIndex - 1 ].position;
            int2 startPosition = abstractPath[ lastIndex ].position;
            int2 firstClusterPosition = startPosition / graphClusterSize;
            int firstClusterIndex = firstClusterPosition.x + firstClusterPosition.y * graphClusterSize;

            if ( graphClustersDense[ firstClusterIndex ] != 0 )
            {
                FixedList4096<int2> lowLevelPath = FindLowLevelPathInCluster( startPosition , toPosition , firstClusterPosition );

                if ( lowLevelPath.Length > 0 )
                {
                    for ( int i = 0; i < lowLevelPath.Length; i++ )
                        concretePath.Add( lowLevelPath[ i ] );
                }
                else
                {
                    //lowLevelPath.Dispose();
                    concretePath.Dispose();
                    return new NativeList<int2>( Allocator.Temp );
                }
            }
            else
            {
                int2 endPosition = abstractPath[ lastIndex ].position;
                concretePath.Add( endPosition );
            }

            #endregion

            return concretePath;
        }
        private void WritePathIndexToUnits( DynamicBuffer<FormationUnit> unitBuffer , int pathIndex )
        {
            /*int i = 0;
            int length = unitBuffer.Length - 4;
            for ( ; i < length; i += 4 )
            {
                pathIndexComponentData[ unitBuffer[ i ].Entity ] = new PathIndex 
                { Value = pathIndex };
                pathIndexComponentData[ unitBuffer[ i + 1 ].Entity ] = new PathIndex
                { Value = pathIndex };
                pathIndexComponentData[ unitBuffer[ i + 2 ].Entity ] = new PathIndex
                { Value = pathIndex };
                pathIndexComponentData[ unitBuffer[ i + 3 ].Entity ] = new PathIndex
                { Value = pathIndex };
            }
            for ( ; i < unitBuffer.Length; i++ )
            {
                pathIndexComponentData[ unitBuffer[ i ].Entity ] = new PathIndex
                { Value = pathIndex };
            }*/
        }
        private bool LOS_Cluster( int2 pos , int2 end )
        {
            int2 dif = math.abs( end - pos );
            int2 inc = new int2(
                ( end.x > pos.x ) ? 1 : -1 ,
                ( end.y > pos.y ) ? 1 : -1 );
            int2 posIncTrue = new int2( inc.x , 0 );
            int2 posIncFalse = new int2( 0 , inc.y );

            int numIterations = 1 + dif.x + dif.y;
            int error = dif.x - dif.y;
            int walkable = 0;

            dif *= 2;

            for ( ; numIterations > 0; --numIterations )
            {
                int2 clusterPosition = pos / graphClusterSize;
                walkable += graphClustersDense[ clusterPosition.x + clusterPosition.y * graphClusterSize ];

                bool branch = error > 0;
                pos += math.select( posIncFalse , posIncTrue , branch );
                error += math.select( dif.x , -dif.y , branch );

                /*if ( error > 0 ) // more steps in x
                {
                    pos.x += inc.x;
                    error -= dif.y;
                }
                else // more steps in y
                {
                    pos.y += inc.y;
                    error += dif.x;
                }*/
            }

            return walkable == 0;
        }
        private bool LOS_Node( int2 pos , int2 end )
        {
            int2 dif = math.abs( end - pos );
            int2 inc = new int2(
                ( end.x > pos.x ) ? 1 : -1 ,
                ( end.y > pos.y ) ? 1 : -1 );
            int2 posIncTrue = new int2( inc.x , 0 );
            int2 posIncFalse = new int2( 0 , inc.y );

            int numIterations = 1 + dif.x + dif.y;
            int error = dif.x - dif.y;
            int walkable = 0;

            dif *= 2;

            for ( ; numIterations > 0; --numIterations )
            {
                walkable += graphNodeWalkables[ pos.x + pos.y * graphCellLength ];

                bool branch = error > 0;
                pos += math.select( posIncFalse , posIncTrue , branch );
                error += math.select( dif.x , -dif.y , branch );

                /*if ( error > 0 ) // more steps in x
                {
                    pos.x += inc.x;
                    error -= dif.y;
                }
                else // more steps in y
                {
                    pos.y += inc.y;
                    error += dif.x;
                }*/
            }

            return walkable == 0;
        }
        private int ManhattenDistance( int2 positionA , int2 positionB )
        {
            int2 distance = positionB - positionA;
            return ( math.abs( distance.x ) + math.abs( distance.y ) );
        }
        #endregion
    }
}