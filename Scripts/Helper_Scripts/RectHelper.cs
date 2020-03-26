using Unity.Mathematics;

public static class RectHelper
{
    public static bool RectanglesTouch( float3 r1_lowerLeft , float3 r1_upperRight , float3 r2_lowerLeft , float3 r2_upperRight )
    {
        return
        RectanglesIntersect( r1_lowerLeft , r1_upperRight , r2_lowerLeft , r2_upperRight ) ||
        RectangleIsWithinOther( r1_lowerLeft , r1_upperRight , r2_lowerLeft , r2_upperRight );
    }

    public static bool RectanglesIntersect( float3 r1_lowerLeft , float3 r1_upperRight , float3 r2_lowerLeft , float3 r2_upperRight )
    {
        return !(
        r1_upperRight.x < r2_lowerLeft.x ||
        r1_upperRight.z < r2_lowerLeft.z ||
        r1_lowerLeft.x > r2_upperRight.x ||
        r1_lowerLeft.z > r2_upperRight.z );
    }

    public static bool RectanglesWithinEachOther( float3 r1_ll , float3 r1_ur , float3 r2_ll , float3 r2_ur )
    {
        return
        RectangleIsWithinOther( r1_ll , r1_ur , r2_ll , r2_ur ) ||
        RectangleIsWithinOther( r2_ll , r2_ur , r1_ll , r1_ur );
    }

    public static bool RectangleIsWithinOther( float3 r1_lowerLeft , float3 r1_upperRight , float3 r2_lowerLeft , float3 r2_upperRight )
    {
        return
        r1_lowerLeft.x >= r2_lowerLeft.x &&
        r1_lowerLeft.z >= r2_lowerLeft.z &&
        r1_upperRight.x <= r2_upperRight.x &&
        r1_upperRight.z <= r2_upperRight.z;
    }

    public static bool PointIsWithinRect( float3 point , float3 r_lowerLeft , float3 r_upperRight )
    {
        return
        point.x >= r_lowerLeft.x &&
        point.z >= r_lowerLeft.z &&
        point.x <= r_upperRight.x &&
        point.z <= r_upperRight.z;
    }
}