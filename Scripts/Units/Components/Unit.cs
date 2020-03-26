using Unity.Entities;
using Unity.Mathematics;

public struct PlatoonReference : IComponentData
{
    public Entity entity;
}

public struct FormationPosition : IComponentData
{
    public int2 position;
}