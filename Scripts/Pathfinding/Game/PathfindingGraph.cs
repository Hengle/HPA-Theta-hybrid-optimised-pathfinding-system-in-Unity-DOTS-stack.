/*using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;

public class PathfindingGraph
{
    public int numCellsAcross;
    public int numClustersAcross;
    public int clusterSize;
    public int cellSize;
    public int gridCapacity;
    public NativeArray<int2> neighbourOffsetArray;

    // CLUSTER
    public NativeArray<int2> clusterEdges;
    public NativeArray<Blittable_Bool> clusterDense;
    public NativeArray<int> intraEdges; // EdgeLength
    public NativeArray<int> interEdges; // EdgeLength
    // EDGE
    public NativeArray<int2> edgeNeighbourKeys; // NodeLength
    public NativeArray<int2> pathKeysPerEdge; // NodeLength
    public NativeArray<int> edgeNeighbours;
    public NativeArray<int2> edgePaths;
    public NativeArray<int2> edgePathPositions;
    // NODE
    public NativeArray<int> nodeToEdge; // NodeLength
    public NativeArray<int2> nodePositions; // NodeLength
    public NativeArray<byte> nodeWalkables; // NodeLength

    public GameObject marker;
    public GameObject searched;

    private PathfindingGrid grid;

    public PathfindingGraph()
    {
        grid = PathfindingGrid.instance;
        marker = grid.GetWaypointMarkerPrefab();
        numCellsAcross = grid.GetCellsAcross();
        numClustersAcross = grid.GetClustersAcross();
        cellSize = grid.GetCellSize();
        clusterSize = grid.GetClusterSize();
        gridCapacity = numCellsAcross * numCellsAcross;

        neighbourOffsetArray = new NativeArray<int2>( 8 , Allocator.Persistent );
        neighbourOffsetArray[ 0 ] = new int2( -1 , 0 );
        neighbourOffsetArray[ 1 ] = new int2( +1 , 0 );
        neighbourOffsetArray[ 2 ] = new int2( 0 , -1 );
        neighbourOffsetArray[ 3 ] = new int2( 0 , +1 );
        neighbourOffsetArray[ 4 ] = new int2( -1 , -1 );
        neighbourOffsetArray[ 5 ] = new int2( +1 , -1 );
        neighbourOffsetArray[ 6 ] = new int2( -1 , +1 );
        neighbourOffsetArray[ 7 ] = new int2( +1 , +1 );

        CreateGraph();

        int count = 0;
        for ( int i = 0; i < edgePathPositions.Length; i++ )
        {
            count++;
            Object.Instantiate( marker , new Vector3( edgePathPositions[ i ].x * cellSize , 2 , edgePathPositions[ i ].y * cellSize ) , Quaternion.identity );
        }

        Debug.Log( count );
    }
    public void Dispose()
    {
        clusterEdges.Dispose();
        clusterDense.Dispose();
        intraEdges.Dispose();
        interEdges.Dispose();
        edgeNeighbourKeys.Dispose();
        pathKeysPerEdge.Dispose();
        edgePaths.Dispose();
        edgeNeighbours.Dispose();
        edgePathPositions.Dispose();
        nodeToEdge.Dispose();
        nodePositions.Dispose();
        //nodeNeighbourLists.Dispose();
        nodeWalkables.Dispose();
        //nodeNeighbours.Dispose();
        neighbourOffsetArray.Dispose();
    }

    private void CreateGraph()
    {
        CreateNodes();
        CreateClusters();
    }

    private void CreateNodes()
    {
        int[] blockedNodes = grid.GetBlockedVerticeArray();

        nodePositions = new NativeArray<int2>( gridCapacity , Allocator.Persistent );
        nodeWalkables = new NativeArray<byte>( gridCapacity , Allocator.Persistent );

        for ( int row = 0; row < numCellsAcross; row++ )
        {
            for ( int col = 0; col < numCellsAcross; col++ )
            {
                int index = col + row * numCellsAcross;

                nodePositions[ index ] = new int2( col , row );
                nodeWalkables[ index ] = 0;

                // Set the blocked nodes
                for ( int i = 0; i < blockedNodes.Length; i++ )
                {
                    if ( index == blockedNodes[ i ] )
                    {
                        nodeWalkables[ index ] = 1;
                        break;
                    }
                }
            }
        }
    }
    private void CreateClusters()
    {
        // Temporary lists for variable-sized 2d -> 1d arrays
        NativeList<int> tempEdgeNeighbours = new NativeList<int>( Allocator.TempJob );
        NativeList<int2> tempEdgePaths = new NativeList<int2>( Allocator.TempJob );
        NativeList<int2> tempEdgePathPositions = new NativeList<int2>( Allocator.TempJob );
        NativeList<int> tempClusterEdges = new NativeList<int>( Allocator.TempJob );
        NativeList<int> tempInterEdges = new NativeList<int>( Allocator.TempJob );

        // Initialize structured arrays
        clusterEdges = new NativeArray<int2>( numClustersAcross * numClustersAcross , Allocator.Persistent );
        clusterDense = new NativeArray<Blittable_Bool>( numClustersAcross * numClustersAcross , Allocator.Persistent );

        for ( int i = 0; i < clusterDense.Length; i++ )
            clusterDense[ i ] = false;

        edgeNeighbourKeys = new NativeArray<int2>( nodePositions.Length , Allocator.Persistent );
        pathKeysPerEdge = new NativeArray<int2>( nodePositions.Length , Allocator.Persistent );
        nodeToEdge = new NativeArray<int>( nodePositions.Length , Allocator.Persistent );

        for ( int i = 0; i < numClustersAcross * numClustersAcross; i++ )
        {   
            int clusterCol = i / numClustersAcross;
            int clusterRow = i % numClustersAcross;
            int x1 = clusterCol * clusterSize;
            int y1 = clusterRow * clusterSize;
            int x2 = x1 + clusterSize;
            int y2 = y1 + clusterSize;

            // Find and create all of the edges in the current cluster
            FindAndCreateEdgesInClusterJob edgeJob = new FindAndCreateEdgesInClusterJob
            {
                startPos = new int2( x1 , y1 ) ,
                endPos = new int2( x2 , y2 ) ,
                numCellsAcross = numCellsAcross ,
                clusterSize = clusterSize ,
                nodeWalkables = nodeWalkables ,
                edgeNodeIndexesInCluster = new NativeList<int2>( Allocator.TempJob )
            };
            edgeJob.Run();
            NativeList<int2> edgeNodeIndexesInCluster = edgeJob.edgeNodeIndexesInCluster;

            // Record the cluster edge indexes
            int startClusterEdgeIndex = tempClusterEdges.Length;
            for ( int j = 0; j < edgeNodeIndexesInCluster.Length; j++ )
            {
                int edgeNodeIndex = edgeNodeIndexesInCluster[ j ].x;
                int interEdgeIndex = edgeNodeIndexesInCluster[ j ].y;

                tempClusterEdges.Add( edgeNodeIndex );
                tempInterEdges.Add( interEdgeIndex );
                nodeToEdge[ edgeNodeIndex ] = tempClusterEdges.Length - 1;
            }

            // Assign the cluster edge key
            int2 clusterEdgeKey = new int2( startClusterEdgeIndex , tempClusterEdges.Length );
            int clusterIndex = clusterCol + clusterRow * numClustersAcross;
            clusterEdges[ clusterIndex ] = clusterEdgeKey;

            // Make all the paths between edges
            for ( int from = 0; from < edgeNodeIndexesInCluster.Length; from++ )
            {
                int startIntraEdgeIndex = tempEdgeNeighbours.Length;
                int startIntraPathIndex = tempEdgePaths.Length;
                int fromNodeIndex = edgeNodeIndexesInCluster[ from ].x;

                for ( int to = 0; to < edgeNodeIndexesInCluster.Length; to++ )
                {
                    if ( from == to )
                        continue;

                    int startTempPositionIndex = tempEdgePathPositions.Length;

                    // DO JOB
                    int toNodeIndex = edgeNodeIndexesInCluster[ to ].x;
                    FindIntraPathJob job = new FindIntraPathJob
                    {
                        neighbourOffsetArray = neighbourOffsetArray ,
                        startNode = fromNodeIndex ,
                        endNode = toNodeIndex ,
                        clusterSize = clusterSize ,
                        numCells = numCellsAcross ,
                        cellSize = cellSize ,
                        clusterPosition = new int2( clusterCol , clusterRow ) ,
                        nodePositions = nodePositions ,
                        nodeWalkables = nodeWalkables ,
                        path = new NativeList<int2>( Allocator.TempJob )
                    };
                    job.Run();
                    NativeList<int2> path = job.path;

                    //Debug.Log( path.Length );

                    // if we have a path
                    if ( path.Length > 0 )
                    {
                        tempEdgeNeighbours.Add( toNodeIndex );

                        for ( int j = 0; j < path.Length; j++ )
                            tempEdgePathPositions.Add( path[ j ] );

                        tempEdgePaths.Add( new int2( startTempPositionIndex , tempEdgePathPositions.Length ) );
                    }

                    path.Dispose();
                }

                edgeNeighbourKeys[ fromNodeIndex ] = new int2( startIntraEdgeIndex , tempEdgeNeighbours.Length );
                pathKeysPerEdge[ fromNodeIndex ] = new int2( startIntraPathIndex , tempEdgePaths.Length );
            }

            edgeNodeIndexesInCluster.Dispose();
        }

        edgeNeighbours = new NativeArray<int>( tempEdgeNeighbours.ToArray() , Allocator.Persistent );
        edgePaths = new NativeArray<int2>( tempEdgePaths.ToArray() , Allocator.Persistent );
        edgePathPositions = new NativeArray<int2>( tempEdgePathPositions.ToArray() , Allocator.Persistent );
        intraEdges = new NativeArray<int>( tempClusterEdges.ToArray() , Allocator.Persistent );
        interEdges = new NativeArray<int>( tempInterEdges.ToArray() , Allocator.Persistent );

        tempEdgeNeighbours.Dispose();
        tempEdgePaths.Dispose();
        tempEdgePathPositions.Dispose();
        tempClusterEdges.Dispose();
        tempInterEdges.Dispose();
    }

    [BurstCompile] private struct FindAndCreateEdgesInClusterJob : IJob
    {
        [ReadOnly] public int2 startPos;
        [ReadOnly] public int2 endPos;
        [ReadOnly] public int numCellsAcross;
        [ReadOnly] public int clusterSize;
        [ReadOnly] public NativeArray<byte> nodeWalkables;

        public NativeList<int2> edgeNodeIndexesInCluster;

        public void Execute()
        {
            // Flip to easily check opposite sides
            int side = -1;

            // Search bottom then top
            for ( int y = startPos.y; y <= endPos.y; y += clusterSize - 1 )
            {
                // If the side of cluster we are searching is not the edge of the grid
                if ( y + side >= 0 && y + side < numCellsAcross )
                {
                    // Get the edges
                    NativeList<int2> horizontalEdges = ( SearchHorizontalEdge( startPos.x , endPos.x , y , side ) );
                    // Store edges
                    for ( int i = 0; i < horizontalEdges.Length; i++ )
                        edgeNodeIndexesInCluster.Add( horizontalEdges[ i ] );
                    horizontalEdges.Dispose();
                }

                side *= -1;
            }

            // Just reseting the flag
            side = -1;

            // Search left then right
            for ( int x = startPos.x; x <= endPos.x; x += clusterSize - 1 )
            {
                // if the side of cluster we are searching is not the edge of the grid
                if ( x + side >= 0 && x + side < numCellsAcross )
                {
                    // Get the edges
                    NativeList<int2> verticalEdges = SearchVerticalEdge( startPos.y , endPos.y , x , side );
                    // Store edges
                    for ( int i = 0; i < verticalEdges.Length; i++ )
                        edgeNodeIndexesInCluster.Add( verticalEdges[ i ] );
                    verticalEdges.Dispose();
                }

                side *= -1;
            }

            // Get the corner edges
            NativeList<int2> cornerEdges = SearchCornerEdge( startPos.x , startPos.y , endPos.x , endPos.y );
            for ( int i = 0; i < cornerEdges.Length; i++ )
                edgeNodeIndexesInCluster.Add( cornerEdges[ i ] );

        }

        private NativeList<int2> SearchHorizontalEdge( int x1 , int x2 , int y , int side )
        {
            // List to return
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );
            int maxClearingSize = ( clusterSize - 1 );
            int startX = x1;
            bool foundClearing = false;

            for ( int x = x1; x < x2; x++ )
            {
                int nodeIndex = x + y * numCellsAcross;
                int interNodeIndex = x + ( y + side ) * numCellsAcross;

                // if the cuurent node and it's neighbour are walkable
                if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ interNodeIndex ] == 0 )
                {
                    // if we havent started tracking an opening, start now
                    if ( !foundClearing )
                    {
                        foundClearing = true;
                        startX = x;
                    }
                    else if ( x - startX >= maxClearingSize ) // if the opening gets too big, just make an edge and start another opening
                    {
                        foundClearing = false;

                        int edgePosX = startX + ( x - startX ) / 2;
                        int intraIndex = edgePosX + y * numCellsAcross;
                        int interIndex = edgePosX + ( y + side ) * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
                else
                {
                    // if they arent walkable and we already have an opening, then determine the edge
                    if ( foundClearing )
                    {
                        foundClearing = false;

                        int edgePosX = startX + ( x - startX ) / 2;
                        int intraIndex = edgePosX + y * numCellsAcross;
                        int interIndex = edgePosX + ( y + side ) * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
            }

            return edgeNodeIndexes;
        }
        private NativeList<int2> SearchVerticalEdge( int y1 , int y2 , int x , int side )
        {
            // HERE WE STORE THE INTERNODES IN THE ARRAY

            // List to return
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );
            int maxClearingSize = ( clusterSize - 1 );
            int startY = y1;
            bool foundClearing = false;

            for ( int y = y1; y < y2; y++ )
            {
                int nodeIndex = x + y * numCellsAcross;
                int interNodeIndex = x + side + y * numCellsAcross;

                // if the cuurent node and it's neighbour are walkable
                if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ interNodeIndex ] == 0 )
                {
                    // if we havent started tracking an opening, start now
                    if ( !foundClearing )
                    {
                        foundClearing = true;
                        startY = y;
                    }
                    else if ( y - startY >= maxClearingSize ) // if the opening gets too big, just make an edge and start another opening
                    {
                        foundClearing = false;

                        int edgePosY = startY + ( y - startY ) / 2;
                        int intraIndex = x + edgePosY * numCellsAcross;
                        int interIndex = x + side + edgePosY * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
                else
                {
                    // if they arent walkable and we already have an opening, then determine the edge
                    if ( foundClearing )
                    {
                        foundClearing = false;

                        int edgePosY = startY + ( y - startY ) / 2;
                        int intraIndex = x + edgePosY * numCellsAcross;
                        int interIndex = x + side + edgePosY * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
            }

            return edgeNodeIndexes;
        }
        private NativeList<int2> SearchCornerEdge( int x1 , int y1 , int x2 , int y2 )
        {
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );

            int xSide = -1;
            int ySide = -1;

            for ( int y = y1; y <= y2; y += clusterSize - 1 )
            {
                for ( int x = x1; x <= x2; x += clusterSize - 1 )
                {
                    if ( x + xSide >= 0 && x + xSide < numCellsAcross && y + ySide >= 0 && y + ySide < numCellsAcross )
                    {
                        int nodeIndex = x + y * numCellsAcross;
                        int n1 = x + xSide + y * numCellsAcross;
                        int n2 = x + xSide + ( y + ySide ) * numCellsAcross;
                        int n3 = x + ( y + ySide ) * numCellsAcross;

                        if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ n1 ] == 0 && nodeWalkables[ n2 ] == 0 && nodeWalkables[ n3 ] == 0 )
                            edgeNodeIndexes.Add( new int2( nodeIndex , n2 ) );
                    }

                    xSide *= -1;
                }

                xSide = -1;
                ySide *= -1;
            }

            return edgeNodeIndexes;
        }
    }
    private struct FindIntraPathJob : IJob
    {
        [ReadOnly] public int startNode;
        [ReadOnly] public int endNode;
        [ReadOnly] public int2 clusterPosition;

        [ReadOnly] public int clusterSize;
        [ReadOnly] public int numCells;
        [ReadOnly] public int cellSize;

        [ReadOnly] public NativeArray<int2> neighbourOffsetArray;
        [ReadOnly] public NativeArray<int2> nodePositions;
        [ReadOnly] public NativeArray<byte> nodeWalkables;

        public NativeList<int2> path;

        public void Execute()
        {
            int pathNodeArraySize = clusterSize * clusterSize;
            int2 startPosition = nodePositions[ startNode ];
            int2 endPosition = nodePositions[ endNode ];

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( pathNodeArraySize , 0 , pathNodeArraySize + 1 );

            NativeArray<int> localIndexArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> graphIndexArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> parentArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( pathNodeArraySize , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( pathNodeArraySize , Allocator.Temp );

            // Initialize nodes from ReadOnly grid array
            for ( int localRow = 0; localRow < clusterSize; localRow++ )
            {
                for ( int localCol = 0; localCol < clusterSize; localCol++ )
                {
                    int localIndex = localCol + localRow * clusterSize;
                    int graphCol = localCol + clusterSize * clusterPosition.x;
                    int graphRow = localRow + clusterSize * clusterPosition.y;
                    int graphArrayIndex = graphCol + graphRow * numCells;

                    localIndexArray[ localIndex ] = localIndex;
                    graphIndexArray[ localIndex ] = graphArrayIndex;
                    parentArray[ localIndex ] = -1;
                    gCostArray[ localIndex ] = int.MaxValue;
                    openArray[ localIndex ] = false;
                    closedArray[ localIndex ] = false;
                }
            }

            // Get and cache the start and end pathNodeIndices
            int endNodeX = endPosition.x - clusterPosition.x * clusterSize;
            int endNodeY = endPosition.y - clusterPosition.y * clusterSize;
            int startNodeX = startPosition.x - clusterPosition.x * clusterSize;
            int startNodeY = startPosition.y - clusterPosition.y * clusterSize;

            int endNodeIndex = endNodeX + endNodeY * clusterSize;
            int startNodeIndex = startNodeX + startNodeY * clusterSize;

            // Initialize the starting pathNode
            int graphIndex = graphIndexArray[ startNodeIndex ];
            int2 nodePosition = nodePositions[ graphIndex ];

            int hCost = ManhattenDistance( nodePosition , endPosition );
            gCostArray[ startNodeIndex ] = 0;
            hCostArray[ startNodeIndex ] = hCost;
            fCostArray[ startNodeIndex ] = hCost;
            openArray[ startNodeIndex ] = true;

            // Add the start pathNode to the openSet
            openSet.Enqueue( startNodeIndex , hCost );

            while ( openSet.Length > 0 )
            {
                // Cache the pathNodeIndex we are working with during this iteration
                int currentNodeIndex = openSet.DequeueMin( localIndexArray , closedArray );

                // Break if we reached our goal
                if ( currentNodeIndex == endNodeIndex )
                    break;

                openArray[ currentNodeIndex ] = false;
                closedArray[ currentNodeIndex ] = true;

                int2 currentNodePosition = nodePositions[ graphIndexArray[ currentNodeIndex ] ];
                for ( int i = 0; i < neighbourOffsetArray.Length; i++ )
                {
                    int2 neighbourPosition = 
                        currentNodePosition + neighbourOffsetArray[ i ];
                    if ( !ValidateGridPosition( neighbourPosition , clusterPosition ) )
                        continue;
                    int graphNeighbour = 
                        neighbourPosition.x + neighbourPosition.y * numCells;
                    if ( nodeWalkables[ graphNeighbour ] > 0 )
                        continue;

                    // Get the local neighbour index
                    int neighbourCol = neighbourPosition.x - clusterPosition.x * clusterSize;
                    int neighbourRow = neighbourPosition.y - clusterPosition.y * clusterSize;
                    int localNeighbourIndex = neighbourCol + neighbourRow * clusterSize;

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

            NativeList<int2> pathFound = new NativeList<int2>( Allocator.Temp );
            NativeList<int2> smoothedPath = new NativeList<int2>( Allocator.Temp );
            int nodeIndex = endNodeIndex;
            bool smoothPath = true;

            if ( parentArray[ nodeIndex ] == -1 )
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
                pathFound.Dispose();
                smoothedPath.Dispose();

                path = new NativeList<int2>( Allocator.Temp );
            }
            else
            {
                pathFound.Add( endPosition );
                smoothedPath.Add( endPosition );

                while ( parentArray[ nodeIndex ] != -1 )
                {
                    pathFound.Add( nodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ] );
                    nodeIndex = parentArray[ nodeIndex ];
                }
            }

            if ( smoothPath && pathFound.Length > 2 ) // If its less than or equal 2 theres no need to smooth the path
            {
                int fromIndex = 0;
                bool foundPath = false;

                while ( !foundPath )
                {
                    int currentIndex = fromIndex + 2; // Because the next index is always going to be in line of sight

                    if ( currentIndex > pathFound.Length - 1 )
                        break;

                    while ( true )
                    {
                        int stopIndex = currentIndex - 1;
                        int graphNodeArrayIndex = pathFound[ stopIndex ].x + pathFound[ stopIndex ].y * numCells;

                        int2 start = pathFound[ fromIndex ];
                        int2 end = pathFound[ currentIndex ];

                        if ( !LOS( start.x , start.y , end.x , end.y ) )
                        {
                            smoothedPath.Add( nodePositions[ graphNodeArrayIndex ] );
                            fromIndex = stopIndex;
                            break;
                        }
                        else
                        {
                            if ( currentIndex >= pathFound.Length - 1 )
                            {
                                foundPath = true;
                                break;
                            }
                            currentIndex++;
                        }
                    }
                }
            }

            openSet.Dispose();
            localIndexArray.Dispose();
            graphIndexArray.Dispose();
            parentArray.Dispose();
            hCostArray.Dispose();
            gCostArray.Dispose();
            fCostArray.Dispose();
            openArray.Dispose();
            closedArray.Dispose();

            path = new NativeList<int2>( pathFound.Length , Allocator.Temp );
            for ( int i = 0; i < pathFound.Length; i++ )
            {
                path[ i ] = pathFound[ i ];
            }

            pathFound.Dispose();
            smoothedPath.Dispose();
        }

        private bool LOS( int x1 , int y1 , int x2 , int y2 )
        {
            int dx = math.abs( x2 - x1 );
            int dy = math.abs( y2 - y1 );
            int x = x1;
            int y = y1;
            int n = 1 + dx + dy;
            int xInc = ( x2 > x1 ) ? 1 : -1;
            int yInc = ( y2 > y1 ) ? 1 : -1;
            int error = dx - dy;
            int walkable = 0;

            dx *= 2;
            dy *= 2;

            for ( ; n > 0; --n )
            {
                walkable += nodeWalkables[ x + y * numCells ];

                if ( error > 0 ) // more steps in x
                {
                    x += xInc;
                    error -= dy;
                }
                else // more steps in y
                {
                    y += yInc;
                    error += dx;
                }
            }

            return walkable == 0;
        }
        private int ManhattenDistance( int2 positionA , int2 positionB )
        {
            int2 distance = positionB - positionA;
            return math.abs( distance.x ) + math.abs( distance.y );
        }
        private bool ValidateGridPosition( int2 position , int2 clusterPosition )
        {
            int2 min = new int2(
                clusterPosition.x * clusterSize ,
                clusterPosition.y * clusterSize );

            return
                position.x >= min.x && position.x < min.x + clusterSize &&
                position.y >= min.y && position.y < min.y + clusterSize;
        }
    }
}*/

using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;

public class PathfindingGraph
{
    public int numCellsAcross;
    public int numClustersAcross;
    public int clusterSize;
    public int cellSize;
    public int gridCapacity;
    public NativeArray<int2> neighbourOffsetArray;

    // CLUSTER
    public NativeArray<int2> clusterEdges;
    public NativeArray<byte> clustersDense;
    public NativeArray<int> intraEdges; // EdgeLength
    public NativeArray<int> interEdges; // EdgeLength
    // EDGE
    public NativeArray<int2> edgeNeighbourKeys; // NodeLength
    public NativeArray<int2> pathKeysPerEdge; // NodeLength
    public NativeArray<int> edgeNeighbours;
    public NativeArray<int2> edgePaths;
    public NativeArray<int2> edgePathNodePositions;
    // NODE
    public NativeArray<int> nodeToEdge; // NodeLength
    public NativeArray<int2> nodePositions; // NodeLength
    public NativeArray<byte> nodeWalkables; // NodeLength

    public GameObject marker;
    public GameObject searched;

    private PathfindingGrid grid;

    public PathfindingGraph()
    {
        grid = PathfindingGrid.instance;
        marker = grid.GetWaypointMarkerPrefab();
        numCellsAcross = grid.GetCellsAcross();
        numClustersAcross = grid.GetClustersAcross();
        cellSize = grid.GetCellSize();
        clusterSize = grid.GetClusterSize();
        gridCapacity = numCellsAcross * numCellsAcross;

        neighbourOffsetArray = new NativeArray<int2>( 8 , Allocator.Persistent );
        neighbourOffsetArray[ 0 ] = new int2( -1 , 0 );
        neighbourOffsetArray[ 1 ] = new int2( +1 , 0 );
        neighbourOffsetArray[ 2 ] = new int2( 0 , -1 );
        neighbourOffsetArray[ 3 ] = new int2( 0 , +1 );
        neighbourOffsetArray[ 4 ] = new int2( -1 , -1 );
        neighbourOffsetArray[ 5 ] = new int2( +1 , -1 );
        neighbourOffsetArray[ 6 ] = new int2( -1 , +1 );
        neighbourOffsetArray[ 7 ] = new int2( +1 , +1 );

        CreateGraph();

        /*for ( int i = 0; i < 50; i++ )
        {
            Object.Instantiate( marker , new Vector3( edgePathNodePositions[ i ].x * cellSize , 2 , edgePathNodePositions[ i ].y * cellSize ) , Quaternion.identity );
        }*/
    }
    public void Dispose()
    {
        clusterEdges.Dispose();
        clustersDense.Dispose();
        intraEdges.Dispose();
        interEdges.Dispose();
        edgeNeighbourKeys.Dispose();
        pathKeysPerEdge.Dispose();
        edgePaths.Dispose();
        edgeNeighbours.Dispose();
        edgePathNodePositions.Dispose();
        nodeToEdge.Dispose();
        nodePositions.Dispose();
        nodeWalkables.Dispose();
        neighbourOffsetArray.Dispose();
    }

    private void CreateGraph()
    {
        CreateNodes();
        CreateClusters();
    }

    private void CreateNodes()
    {
        int[] blockedNodes = grid.GetBlockedVerticeArray();

        nodePositions = new NativeArray<int2>( gridCapacity , Allocator.Persistent );
        nodeWalkables = new NativeArray<byte>( gridCapacity , Allocator.Persistent );

        for ( int row = 0; row < numCellsAcross; row++ )
        {
            for ( int col = 0; col < numCellsAcross; col++ )
            {
                int index = col + row * numCellsAcross;

                nodePositions[ index ] = new int2( col , row );
                nodeWalkables[ index ] = 0;

                // Set the blocked nodes
                for ( int i = 0; i < blockedNodes.Length; i++ )
                {
                    if ( index == blockedNodes[ i ] )
                    {
                        nodeWalkables[ index ] = 1;
                        break;
                    }
                }
            }
        }
    }
    private void CreateClusters()
    {
        // Temporary lists for variable-sized 2d -> 1d arrays
        NativeList<int> tempEdgeNeighbours = new NativeList<int>( Allocator.TempJob );
        NativeList<int2> tempEdgePaths = new NativeList<int2>( Allocator.TempJob );
        NativeList<int2> tempEdgePathPositions = new NativeList<int2>( Allocator.TempJob );
        NativeList<int> tempClusterEdges = new NativeList<int>( Allocator.TempJob );
        NativeList<int> tempInterEdges = new NativeList<int>( Allocator.TempJob );

        // Initialize structured arrays
        clusterEdges = new NativeArray<int2>( numClustersAcross * numClustersAcross , Allocator.Persistent );
        clustersDense = new NativeArray<byte>( numClustersAcross * numClustersAcross , Allocator.Persistent );
        edgeNeighbourKeys = new NativeArray<int2>( nodePositions.Length , Allocator.Persistent );
        pathKeysPerEdge = new NativeArray<int2>( nodePositions.Length , Allocator.Persistent );
        nodeToEdge = new NativeArray<int>( nodePositions.Length , Allocator.Persistent );

        for ( int i = 0; i < numCellsAcross * numCellsAcross; i++ )
        {
            if ( nodeWalkables[ i ] == 1 )
            {
                int2 nodePosition = nodePositions[ i ];
                int2 clusterPosition = new int2(
                    nodePosition.x / numClustersAcross ,
                    nodePosition.y / numClustersAcross );
                int clusterIndex = clusterPosition.x + clusterPosition.y * numClustersAcross;
                clustersDense[ clusterIndex ] = 1;
            }
        }

        /*for ( int i = 0; i < clustersDense.Length; i++ )
            clustersDense[ i ] = 0;*/

        for ( int i = 0; i < numClustersAcross * numClustersAcross; i++ )
        {
            int clusterCol = i / numClustersAcross;
            int clusterRow = i % numClustersAcross;
            int x1 = clusterCol * clusterSize;
            int y1 = clusterRow * clusterSize;
            int x2 = x1 + clusterSize;
            int y2 = y1 + clusterSize;

            // Find and create all of the edges in the current cluster
            FindAndCreateEdgesInClusterJob edgeJob = new FindAndCreateEdgesInClusterJob
            {
                startPos = new int2( x1 , y1 ) ,
                endPos = new int2( x2 , y2 ) ,
                numCellsAcross = numCellsAcross ,
                clusterSize = clusterSize ,
                nodeWalkables = nodeWalkables ,
                edgeNodeIndexesInCluster = new NativeList<int2>( Allocator.TempJob )
            };
            edgeJob.Run();
            NativeList<int2> edgeNodeIndexesInCluster = edgeJob.edgeNodeIndexesInCluster;

            // Record the cluster edge indexes
            int startClusterEdgeIndex = tempClusterEdges.Length;
            for ( int j = 0; j < edgeNodeIndexesInCluster.Length; j++ )
            {
                int edgeNodeIndex = edgeNodeIndexesInCluster[ j ].x;
                int interEdgeIndex = edgeNodeIndexesInCluster[ j ].y;

                tempClusterEdges.Add( edgeNodeIndex );
                tempInterEdges.Add( interEdgeIndex );
                nodeToEdge[ edgeNodeIndex ] = tempClusterEdges.Length - 1;
            }

            // Assign the cluster edge key
            int2 clusterEdgeKey = new int2( startClusterEdgeIndex , tempClusterEdges.Length );
            int clusterIndex = clusterCol + clusterRow * numClustersAcross;
            clusterEdges[ clusterIndex ] = clusterEdgeKey;

            // Make all the paths between edges
            for ( int from = 0; from < edgeNodeIndexesInCluster.Length; from++ )
            {
                int startIntraEdgeIndex = tempEdgeNeighbours.Length;
                int startIntraPathIndex = tempEdgePaths.Length;
                int fromNodeIndex = edgeNodeIndexesInCluster[ from ].x;

                for ( int to = 0; to < edgeNodeIndexesInCluster.Length; to++ )
                {
                    if ( from == to )
                        continue;

                    int startTempPositionIndex = tempEdgePathPositions.Length;

                    // DO JOB
                    int toNodeIndex = edgeNodeIndexesInCluster[ to ].x;
                    FindIntraPathJob job = new FindIntraPathJob
                    {
                        neighbourOffsetArray = neighbourOffsetArray ,
                        startNode = fromNodeIndex ,
                        endNode = toNodeIndex ,
                        clusterSize = clusterSize ,
                        numCells = numCellsAcross ,
                        cellSize = cellSize ,
                        clusterPosition = new int2( clusterCol , clusterRow ) ,
                        nodePositions = nodePositions ,
                        nodeWalkables = nodeWalkables ,
                        path = new NativeList<int2>( Allocator.TempJob )
                    };
                    job.Run();
                    NativeList<int2> path = job.path;

                    // if we have a path
                    if ( path.Length > 0 )
                    {
                        tempEdgeNeighbours.Add( toNodeIndex );

                        for ( int j = 0; j < path.Length; j++ )
                            tempEdgePathPositions.Add( path[ j ] );

                        tempEdgePaths.Add( new int2( startTempPositionIndex , tempEdgePathPositions.Length ) );
                    }

                    path.Dispose();
                }

                edgeNeighbourKeys[ fromNodeIndex ] = new int2( startIntraEdgeIndex , tempEdgeNeighbours.Length );
                pathKeysPerEdge[ fromNodeIndex ] = new int2( startIntraPathIndex , tempEdgePaths.Length );
            }

            edgeNodeIndexesInCluster.Dispose();
        }

        edgeNeighbours = new NativeArray<int>( tempEdgeNeighbours.ToArray() , Allocator.Persistent );
        edgePaths = new NativeArray<int2>( tempEdgePaths.ToArray() , Allocator.Persistent );
        edgePathNodePositions = new NativeArray<int2>( tempEdgePathPositions.ToArray() , Allocator.Persistent );
        intraEdges = new NativeArray<int>( tempClusterEdges.ToArray() , Allocator.Persistent );
        interEdges = new NativeArray<int>( tempInterEdges.ToArray() , Allocator.Persistent );

        tempEdgeNeighbours.Dispose();
        tempEdgePaths.Dispose();
        tempEdgePathPositions.Dispose();
        tempClusterEdges.Dispose();
        tempInterEdges.Dispose();
    }

    [BurstCompile]
    private struct FindAndCreateEdgesInClusterJob : IJob
    {
        [ReadOnly] public int2 startPos;
        [ReadOnly] public int2 endPos;
        [ReadOnly] public int numCellsAcross;
        [ReadOnly] public int clusterSize;
        [ReadOnly] public NativeArray<byte> nodeWalkables;

        public NativeList<int2> edgeNodeIndexesInCluster;

        public void Execute()
        {
            // Flip to easily check opposite sides
            int side = -1;

            // Search bottom then top
            for ( int y = startPos.y; y <= endPos.y; y += clusterSize - 1 )
            {
                // If the side of cluster we are searching is not the edge of the grid
                if ( y + side >= 0 && y + side < numCellsAcross )
                {
                    // Get the edges
                    NativeList<int2> horizontalEdges = ( SearchHorizontalEdge( startPos.x , endPos.x , y , side ) );
                    // Store edges
                    for ( int i = 0; i < horizontalEdges.Length; i++ )
                        edgeNodeIndexesInCluster.Add( horizontalEdges[ i ] );
                    horizontalEdges.Dispose();
                }

                side *= -1;
            }

            // Just reseting the flag
            side = -1;

            // Search left then right
            for ( int x = startPos.x; x <= endPos.x; x += clusterSize - 1 )
            {
                // if the side of cluster we are searching is not the edge of the grid
                if ( x + side >= 0 && x + side < numCellsAcross )
                {
                    // Get the edges
                    NativeList<int2> verticalEdges = SearchVerticalEdge( startPos.y , endPos.y , x , side );
                    // Store edges
                    for ( int i = 0; i < verticalEdges.Length; i++ )
                        edgeNodeIndexesInCluster.Add( verticalEdges[ i ] );
                    verticalEdges.Dispose();
                }

                side *= -1;
            }

            // Get the corner edges
            NativeList<int2> cornerEdges = SearchCornerEdge( startPos.x , startPos.y , endPos.x , endPos.y );
            for ( int i = 0; i < cornerEdges.Length; i++ )
                edgeNodeIndexesInCluster.Add( cornerEdges[ i ] );

        }

        private NativeList<int2> SearchHorizontalEdge( int x1 , int x2 , int y , int side )
        {
            // List to return
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );
            int maxClearingSize = ( clusterSize - 1 );
            int startX = x1;
            bool foundClearing = false;

            for ( int x = x1; x < x2; x++ )
            {
                int nodeIndex = x + y * numCellsAcross;
                int interNodeIndex = x + ( y + side ) * numCellsAcross;

                // if the cuurent node and it's neighbour are walkable
                if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ interNodeIndex ] == 0 )
                {
                    // if we havent started tracking an opening, start now
                    if ( !foundClearing )
                    {
                        foundClearing = true;
                        startX = x;
                    }
                    else if ( x - startX >= maxClearingSize ) // if the opening gets too big, just make an edge and start another opening
                    {
                        foundClearing = false;

                        int edgePosX = startX + ( x - startX ) / 2;
                        int intraIndex = edgePosX + y * numCellsAcross;
                        int interIndex = edgePosX + ( y + side ) * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
                else
                {
                    // if they arent walkable and we already have an opening, then determine the edge
                    if ( foundClearing )
                    {
                        foundClearing = false;

                        int edgePosX = startX + ( x - startX ) / 2;
                        int intraIndex = edgePosX + y * numCellsAcross;
                        int interIndex = edgePosX + ( y + side ) * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
            }

            return edgeNodeIndexes;
        }
        private NativeList<int2> SearchVerticalEdge( int y1 , int y2 , int x , int side )
        {
            // HERE WE STORE THE INTERNODES IN THE ARRAY

            // List to return
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );
            int maxClearingSize = ( clusterSize - 1 );
            int startY = y1;
            bool foundClearing = false;

            for ( int y = y1; y < y2; y++ )
            {
                int nodeIndex = x + y * numCellsAcross;
                int interNodeIndex = x + side + y * numCellsAcross;

                // if the cuurent node and it's neighbour are walkable
                if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ interNodeIndex ] == 0 )
                {
                    // if we havent started tracking an opening, start now
                    if ( !foundClearing )
                    {
                        foundClearing = true;
                        startY = y;
                    }
                    else if ( y - startY >= maxClearingSize ) // if the opening gets too big, just make an edge and start another opening
                    {
                        foundClearing = false;

                        int edgePosY = startY + ( y - startY ) / 2;
                        int intraIndex = x + edgePosY * numCellsAcross;
                        int interIndex = x + side + edgePosY * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
                else
                {
                    // if they arent walkable and we already have an opening, then determine the edge
                    if ( foundClearing )
                    {
                        foundClearing = false;

                        int edgePosY = startY + ( y - startY ) / 2;
                        int intraIndex = x + edgePosY * numCellsAcross;
                        int interIndex = x + side + edgePosY * numCellsAcross;

                        edgeNodeIndexes.Add( new int2( intraIndex , interIndex ) );
                    }
                }
            }

            return edgeNodeIndexes;
        }
        private NativeList<int2> SearchCornerEdge( int x1 , int y1 , int x2 , int y2 )
        {
            NativeList<int2> edgeNodeIndexes = new NativeList<int2>( Allocator.Temp );

            int xSide = -1;
            int ySide = -1;

            for ( int y = y1; y <= y2; y += clusterSize - 1 )
            {
                for ( int x = x1; x <= x2; x += clusterSize - 1 )
                {
                    if ( x + xSide >= 0 && x + xSide < numCellsAcross && y + ySide >= 0 && y + ySide < numCellsAcross )
                    {
                        int nodeIndex = x + y * numCellsAcross;
                        int n1 = x + xSide + y * numCellsAcross;
                        int n2 = x + xSide + ( y + ySide ) * numCellsAcross;
                        int n3 = x + ( y + ySide ) * numCellsAcross;

                        if ( nodeWalkables[ nodeIndex ] == 0 && nodeWalkables[ n1 ] == 0 && nodeWalkables[ n2 ] == 0 && nodeWalkables[ n3 ] == 0 )
                            edgeNodeIndexes.Add( new int2( nodeIndex , n2 ) );
                    }

                    xSide *= -1;
                }

                xSide = -1;
                ySide *= -1;
            }

            return edgeNodeIndexes;
        }
    }
    [BurstCompile]
    private struct FindIntraPathJob : IJob
    {
        [ReadOnly] public int startNode;
        [ReadOnly] public int endNode;
        [ReadOnly] public int2 clusterPosition;

        [ReadOnly] public int clusterSize;
        [ReadOnly] public int numCells;
        [ReadOnly] public int cellSize;

        [ReadOnly] public NativeArray<int2> neighbourOffsetArray;
        [ReadOnly] public NativeArray<int2> nodePositions;
        [ReadOnly] public NativeArray<byte> nodeWalkables;

        public NativeList<int2> path;

        public void Execute()
        {
            int pathNodeArraySize = clusterSize * clusterSize;
            int2 startPosition = nodePositions[ startNode ];
            int2 endPosition = nodePositions[ endNode ];

            // Node sets
            NativeMinHeap openSet = new NativeMinHeap();
            openSet.Initialize( pathNodeArraySize , 0 , pathNodeArraySize + 1 );

            NativeArray<int> localIndexArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> graphIndexArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> parentArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> hCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> gCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<int> fCostArray = new NativeArray<int>( pathNodeArraySize , Allocator.Temp );
            NativeArray<Blittable_Bool> openArray = new NativeArray<Blittable_Bool>( pathNodeArraySize , Allocator.Temp );
            NativeArray<Blittable_Bool> closedArray = new NativeArray<Blittable_Bool>( pathNodeArraySize , Allocator.Temp );

            // Initialize nodes from ReadOnly grid array
            for ( int localRow = 0; localRow < clusterSize; localRow++ )
            {
                for ( int localCol = 0; localCol < clusterSize; localCol++ )
                {
                    int localIndex = localCol + localRow * clusterSize;
                    int graphCol = localCol + clusterSize * clusterPosition.x;
                    int graphRow = localRow + clusterSize * clusterPosition.y;
                    int graphArrayIndex = graphCol + graphRow * numCells;

                    localIndexArray[ localIndex ] = localIndex;
                    graphIndexArray[ localIndex ] = graphArrayIndex;
                    parentArray[ localIndex ] = -1;
                    gCostArray[ localIndex ] = int.MaxValue;
                    openArray[ localIndex ] = false;
                    closedArray[ localIndex ] = false;
                }
            }

            // Get and cache the start and end pathNodeIndices
            int endNodeX = endPosition.x - clusterPosition.x * clusterSize;
            int endNodeY = endPosition.y - clusterPosition.y * clusterSize;
            int startNodeX = startPosition.x - clusterPosition.x * clusterSize;
            int startNodeY = startPosition.y - clusterPosition.y * clusterSize;

            int endNodeIndex = endNodeX + endNodeY * clusterSize;
            int startNodeIndex = startNodeX + startNodeY * clusterSize;

            // Initialize the starting pathNode
            int graphIndex = graphIndexArray[ startNodeIndex ];
            int2 nodePosition = nodePositions[ graphIndex ];

            int hCost = ManhattenDistance( nodePosition , endPosition );
            gCostArray[ startNodeIndex ] = 0;
            hCostArray[ startNodeIndex ] = hCost;
            fCostArray[ startNodeIndex ] = hCost;
            openArray[ startNodeIndex ] = true;

            // Add the start pathNode to the openSet
            openSet.Enqueue( startNodeIndex , hCost );

            int count = 0;
            while ( openSet.Length > 0 )
            {
                count++;
                if ( count > 1000 )
                    break;
                // Cache the pathNodeIndex we are working with during this iteration
                int currentNodeIndex = openSet.DequeueMin( localIndexArray , closedArray );

                // Break if we reached our goal
                if ( currentNodeIndex == endNodeIndex )
                    break;

                openArray[ currentNodeIndex ] = false;
                closedArray[ currentNodeIndex ] = true;

                int2 currentNodePosition = nodePositions[ graphIndexArray[ currentNodeIndex ] ];
                for ( int i = 0; i < neighbourOffsetArray.Length; i++ )
                {
                    int2 neighbourPosition = currentNodePosition + neighbourOffsetArray[ i ];

                    if ( !ValidateGridPosition( neighbourPosition , clusterPosition ) )
                        continue;

                    int graphNeighbour = neighbourPosition.x + neighbourPosition.y * numCells;

                    if ( nodeWalkables[ graphNeighbour ] > 0 )
                        continue;

                    // Get the local neighbour index
                    int neighbourCol = neighbourPosition.x - clusterPosition.x * clusterSize;
                    int neighbourRow = neighbourPosition.y - clusterPosition.y * clusterSize;
                    int localNeighbourIndex = neighbourCol + neighbourRow * clusterSize;

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

            NativeList<int2> aStarPath = new NativeList<int2>( Allocator.Temp );
            int nodeIndex = endNodeIndex;
            bool smoothPath = true;

            if ( parentArray[ nodeIndex ] == -1 )
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
                aStarPath.Dispose();

                path = new NativeList<int2>( Allocator.Temp );
            }
            else
            {
                path.Add( endPosition );
                aStarPath.Add( endPosition );

                while ( parentArray[ nodeIndex ] != -1 )
                {
                    aStarPath.Add( nodePositions[ graphIndexArray[ parentArray[ nodeIndex ] ] ] );
                    nodeIndex = parentArray[ nodeIndex ];
                }

                openSet.Dispose();
                localIndexArray.Dispose();
                graphIndexArray.Dispose();
                parentArray.Dispose();
                hCostArray.Dispose();
                gCostArray.Dispose();
                fCostArray.Dispose();
                openArray.Dispose();
                closedArray.Dispose();

                //for ( int i = 0; i < aStarPath.Length; i++ )
                //path.Add( aStarPath[ i ] );
            }

            if ( smoothPath && aStarPath.Length > 2 ) // If its less than or equal 2 theres no need to smooth the path
            {
                int fromIndex = 0;
                bool foundPath = false;

                while ( !foundPath )
                {
                    int currentIndex = fromIndex + 2; // Because the next index is always going to be in line of sight

                    if ( currentIndex > aStarPath.Length - 1 )
                        break;

                    while ( true )
                    {
                        int stopIndex = currentIndex - 1;
                        int graphNodeArrayIndex = aStarPath[ stopIndex ].x + aStarPath[ stopIndex ].y * numCells;

                        int2 start = aStarPath[ fromIndex ];
                        int2 end = aStarPath[ currentIndex ];

                        if ( !LOS( start.x , start.y , end.x , end.y ) )
                        {
                            path.Add( nodePositions[ graphNodeArrayIndex ] );
                            fromIndex = stopIndex;
                            break;
                        }
                        else
                        {
                            if ( currentIndex >= aStarPath.Length - 1 )
                            {
                                foundPath = true;
                                break;
                            }
                            currentIndex++;
                        }
                    }
                }
            }

            aStarPath.Dispose();

            path.Add( startPosition );
        }

        private bool LOS( int x1 , int y1 , int x2 , int y2 )
        {
            int dx = math.abs( x2 - x1 );
            int dy = math.abs( y2 - y1 );
            int x = x1;
            int y = y1;
            int n = 1 + dx + dy;
            int xInc = ( x2 > x1 ) ? 1 : -1;
            int yInc = ( y2 > y1 ) ? 1 : -1;
            int error = dx - dy;
            int walkable = 0;

            bool prevY = false;

            if ( error <= 0 )
            {
                prevY = true;
            }

            int numSameDir = 0;

            dx *= 2;
            dy *= 2;

            for ( ; n > 0; --n )
            {
                walkable += nodeWalkables[ x + y * numCells ];

                if ( numSameDir < 1 )
                {
                    if ( prevY )
                    {
                        walkable += nodeWalkables[ ( x - xInc ) + y * numCells ];

                    }
                    else
                    {
                        walkable += nodeWalkables[ x + ( y - yInc ) * numCells ];
                    }
                }

                if ( error > 0 ) // more steps in x
                {
                    x += xInc;

                    if ( !prevY )
                    {
                        numSameDir++;
                    }
                    else
                    {
                        numSameDir = 0;
                    }

                    prevY = false;
                    error -= dy;
                }
                else // more steps in y
                {
                    y += yInc;

                    if ( prevY )
                    {
                        numSameDir++;
                    }
                    else
                    {
                        numSameDir = 0;
                    }

                    prevY = true;
                    error += dx;
                }
            }

            return walkable == 0;
        }
        private int ManhattenDistance( int2 positionA , int2 positionB )
        {
            int2 distance = positionB - positionA;
            return math.abs( distance.x ) + math.abs( distance.y );
        }
        private bool ValidateGridPosition( int2 position , int2 clusterPosition )
        {
            int2 min = new int2(
                clusterPosition.x * clusterSize ,
                clusterPosition.y * clusterSize );

            return
                position.x >= min.x && position.x < min.x + clusterSize &&
                position.y >= min.y && position.y < min.y + clusterSize;
        }
    }
}