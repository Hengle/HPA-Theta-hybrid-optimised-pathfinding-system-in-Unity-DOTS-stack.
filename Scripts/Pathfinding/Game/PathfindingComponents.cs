using Unity.Entities;
using Unity.Mathematics;

public struct SelectedTag : IComponentData
{
}
public struct PathFindingOrders : IComponentData
{
    public float2 targetPosition;
}
public struct PathIndex : IComponentData
{
    public int index;
}
[InternalBufferCapacity( 30 )]
public struct PathPosition : IBufferElementData
{
    public float2 position;
}