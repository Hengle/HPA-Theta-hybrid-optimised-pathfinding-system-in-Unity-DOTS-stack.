public struct Blittable_Bool
{
    public readonly byte Value;

    public Blittable_Bool( byte value )
    {
        Value = value;
    }

    public Blittable_Bool( bool value )
    {
        Value = value ? ( byte ) 1 : ( byte ) 0;
    }

    public static implicit operator bool( Blittable_Bool bb )
    {
        return bb.Value != 0;
    }

    public static implicit operator Blittable_Bool( bool b )
    {
        return new Blittable_Bool( b );
    }
}
