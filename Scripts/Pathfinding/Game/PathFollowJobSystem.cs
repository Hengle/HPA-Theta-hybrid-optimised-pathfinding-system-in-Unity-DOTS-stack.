using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections;
using Unity.Transforms;

public class PathFollowJobSystem : JobComponentSystem
{
    [BurstCompile] private struct PathFollowJob : IJobForEachWithEntity_EBCC<PathPosition , PathIndex , Translation>
    {
        [ReadOnly] public float dt;

        public void Execute( Entity entity , int index , 
            DynamicBuffer<PathPosition> pathPositionBuffer , ref PathIndex pathIndex , ref Translation translation )
        {
            if ( pathIndex.index >= 0 )
            {
                float3 pathPosition = new float3( 
                    pathPositionBuffer[ pathIndex.index ].position.x , 2 , pathPositionBuffer[ pathIndex.index ].position.y );
                float distance = math.distance( 
                    pathPosition , translation.Value );
                float3 direction = math.normalize( 
                    pathPosition - translation.Value );

                if ( math.abs( direction.x ) <= 1 )
                    translation.Value += new float3( 
                        direction.x * 30f * dt , 0 , direction.z * 30f * dt );

                if ( distance <= 0.125f )
                {
                    int newPathIndex = pathIndex.index - 1;
                    pathIndex = new PathIndex { index = newPathIndex };
                }
            }
        }
    }
    
    protected override JobHandle OnUpdate( JobHandle inputDeps )
    {
        PathFollowJob job = new PathFollowJob
        {
            dt = Time.DeltaTime
        };
        JobHandle jobHandle = job.Schedule( this , inputDeps );

        return jobHandle;
    }
}