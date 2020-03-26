using Unity.Collections;

public struct NativeMinHeap
{
    private struct Element
    {
        public int key;
        public int value;
    }

    private NativeArray<Element> data;
    private int capacity;
    private int supremum;
    private int size;

    public int Length => size;

    public void Initialize( int capacity , int infimum , int supremum )
    {
        this.data = new NativeArray<Element>( capacity + 2 , Allocator.Temp );
        this.capacity = capacity;
        this.supremum = supremum;
        this.size = 0;

        SetKey( 0 , infimum );
        SetKey( capacity + 1 , supremum );
        Clear();
    }
    public void Enqueue( int index , int priority )
    {
        Enqueue_Impl( index , priority );
    }
    public int DequeueMin1( NativeArray<PathNode> pathNodes )
    {
        PathNode node;
        do
        {
            node = pathNodes[ Dequeue_Impl() ];
        }
        while ( node.isClosed );
        return node.localArrayIndex;
    }
    public int DequeueMin( NativeArray<int> indexArray , NativeArray<Blittable_Bool> closedArray )
    {
        int i;
        do i = Dequeue_Impl();
        while ( closedArray[ i ] );
        return indexArray[ i ];
    }
    public int DequeueMin( NativeArray<Blittable_Bool> closedArray )
    {
        int i;
        do i = Dequeue_Impl();
        while ( closedArray[ i ] );
        return i;
    }
    public void Dispose()
    {
        data.Dispose();
    }

    private void SetKey( int index , int key )
    {
        var item = data[ index ];
        item.key = key;
        data[ index ] = item;
    }
    private void SetKeyValue( int index , int key , int value )
    {
        data[ index ] = new Element() { key = key , value = value };
    }
    private void Clear()
    {
        size = 0;
        int cap = capacity;
        for ( int i = 1; i <= cap; ++i )
        {
            SetKeyValue( i , supremum , default( int ) );
        }
    }
    private void Enqueue_Impl( int value , int key )
    {
        ++size;
        int hole = size;
        int pred = hole >> 1;
        int predKey = data[ pred ].key;
        while ( predKey.CompareTo( key ) > 0 )
        {
            SetKeyValue( hole , predKey , data[ pred ].value );
            hole = pred;
            pred >>= 1;
            predKey = data[ pred ].key;
        }

        SetKeyValue( hole , key , value );
    }
    private int Dequeue_Impl()
    {
        int min = data[ 1 ].value;

        int hole = 1;
        int succ = 2;
        int sz = size;

        while ( succ < sz )
        {
            int key1 = data[ succ ].key;
            int key2 = data[ succ + 1 ].key;
            if ( key1.CompareTo( key2 ) > 0 )
            {
                succ++;
                SetKeyValue( hole , key2 , data[ succ ].value );
            }
            else
            {
                SetKeyValue( hole , key1 , data[ succ ].value );
            }
            hole = succ;
            succ <<= 1;
        }

        int bubble = data[ sz ].key;
        int pred = hole >> 1;
        while ( data[ pred ].key.CompareTo( bubble ) > 0 )
        {
            data[ hole ] = data[ pred ];
            hole = pred;
            pred >>= 1;
        }

        SetKeyValue( hole , bubble , data[ sz ].value );

        SetKey( size , supremum );
        size = sz - 1;

        return min;
    }
}

public struct PathNode
{
    public int graphArrayIndex;
    public int localArrayIndex;
    public int parent;

    public int hCost;
    public int gCost;
    public int fCost;

    public Blittable_Bool isClosed;
    public Blittable_Bool isOpen;
}

public struct SetFlag
{
    public Blittable_Bool closed;
    public Blittable_Bool open;
}