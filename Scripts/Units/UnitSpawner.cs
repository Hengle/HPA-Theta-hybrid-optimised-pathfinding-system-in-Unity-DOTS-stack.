using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Rendering;
using Unity.Transforms;

public class UnitSpawner
{
    private const int NUMBER_OF_UNITS_WIDE = 20;
    private const int NUMBER_OF_UNITS_LONG = 20;
    private const int UNIT_SPACING = 2;

    private EntityManager entityManager;

    public UnitSpawner( Mesh mesh , Material material )
    {
        entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;

        for ( int i = 0; i < NUMBER_OF_UNITS_LONG; i++ )
        {
            for ( int j = 0; j < NUMBER_OF_UNITS_WIDE; j++ )
            {
                float3 position = new float3( ( j + 1 ) * UNIT_SPACING * 2 + 6 , 2 , ( i + 1 ) * UNIT_SPACING * 2 + 6 );
                float scale = 2;

                Entity unit = entityManager.CreateEntity(
                    typeof( RenderMesh ) , typeof( RenderBounds ) , typeof( LocalToWorld ) ,
                    typeof( Translation ) , typeof( Scale ) , typeof( PathIndex ) , typeof( PathPosition ) );

                entityManager.SetSharedComponentData( unit , new RenderMesh {
                    material = material ,
                    mesh = mesh
                } );
                entityManager.SetComponentData( unit , new Translation { Value = position } );
                entityManager.SetComponentData( unit , new Scale { Value = scale } );
                entityManager.SetComponentData( unit , new PathIndex { index = -1 } );
            }
        }
    }
}
