using Unity.Entities;
using Unity.Mathematics;

public struct CompanyReference : IBufferElementData
{
    public Entity entity;
}

[InternalBufferCapacity(30)]
public struct UnitBuffer : IBufferElementData
{
    public Entity entity;
}

public struct Formation : IComponentData
{
    public int2 dimensions;
}