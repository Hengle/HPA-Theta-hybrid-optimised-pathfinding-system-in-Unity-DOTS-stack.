using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[AlwaysUpdateSystem]
public class InputSystem : ComponentSystem
{
    private float3 mouseStart = float3.zero;

    private CameraController cameraController;
    private double doubleClickTime = 0;
    private float doubleClickDelay = 0.5f;
    private int clicked = 0;

    private double leftHoldTime = 0;
    private float leftHoldDelay = 0.25f;

    protected override void OnStartRunning()
    {
        base.OnStartRunning();
        cameraController = Camera.main.GetComponent<CameraController>();
    }
    protected override void OnUpdate()
    {
        if ( Input.GetMouseButtonDown( 0 ) )
            On_LeftDown();
        else if ( Input.GetMouseButton( 0 ) )
            On_LeftHold();
        else if ( Input.GetMouseButtonUp( 0 ) )
            On_LeftUp();
        else if ( Input.GetMouseButtonDown( 1 ) )
            On_RightDown();
        else if ( Input.GetMouseButton( 1 ) )
            On_RightHold();
        else if ( Input.GetMouseButtonUp( 1 ) )
            On_RightUp();
    }

    private void On_LeftDown()
    {
        float3 mousePosition = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            mouseStart = new float3( hit.point.x , hit.point.y , hit.point.z );
            leftHoldTime = Time.ElapsedTime;

            clicked++;
            if ( clicked == 1 )
            {
                doubleClickTime = Time.ElapsedTime;
            }
        }

        Entities.WithAll<SelectedTag>().ForEach( ( Entity entity ) =>
        {
            PostUpdateCommands.RemoveComponent<SelectedTag>( entity );
        } );

        if ( clicked > 1 && Time.ElapsedTime - doubleClickTime < doubleClickDelay )
        {
            clicked = 0;
            doubleClickTime = 0;
            cameraController.ZoomToLocation( mouseStart );
        }
        else if ( clicked > 2 || Time.ElapsedTime - doubleClickTime > doubleClickDelay )
        {
            clicked = 0;
        }
    }
    private void On_LeftHold()
    {
        if ( Time.ElapsedTime - leftHoldTime > leftHoldDelay )
        {
            //GameHandler.Instance.mouse_startPos = mouseStart;
            //GameHandler.Instance.mouse_endPos = Input.mousePosition;
            //GameHandler.Instance.showBox = true;
        }
    }
    private void On_LeftUp()
    {
        float3 mousePosition = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            mousePosition = new float3( hit.point.x , hit.point.y , hit.point.z );

            float xSize = math.abs( mousePosition.x - mouseStart.x );
            float zSize = math.abs( mousePosition.z - mouseStart.z );

            if ( xSize > 5f && zSize > 5f )
            {
                float3 startPos = mouseStart;
                float3 endPos = mousePosition;

                if ( startPos.x > endPos.x )
                {
                    float temp = startPos.x;
                    startPos.x = endPos.x;
                    endPos.x = temp;
                }
                if ( startPos.z > endPos.z )
                {
                    float temp = startPos.z;
                    startPos.z = endPos.z;
                    endPos.z = temp;
                }

                Drag_Select( startPos , endPos );
            }
            else
            {
                Click_Select( mousePosition );
            }
        }
    }

    private void On_RightDown()
    {
        float3 mousePosition = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            mouseStart = new float3( hit.point.x , hit.point.y , hit.point.z );
        }
    }
    private void On_RightHold()
    {
        float3 mousePosition = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            mousePosition = new float3( hit.point.x , hit.point.y , hit.point.z );
            Debug.DrawLine( mouseStart + new float3( 0 , 5 , 0 ) , mousePosition + new float3( 0 , 5 , 0 ) , Color.blue );
        }
    }
    private void On_RightUp()
    {
        float3 mousePosition = Input.mousePosition;
        Ray ray = Camera.main.ScreenPointToRay( mousePosition );

        if ( Physics.Raycast( ray , out RaycastHit hit ) )
        {
            mousePosition = new float3( hit.point.x , hit.point.y , hit.point.z );

            float xSize = math.abs( mousePosition.x - mouseStart.x );
            float zSize = math.abs( mousePosition.z - mouseStart.z );

            if ( xSize < 2 && zSize < 2 )
            {
                Entities.WithAll<SelectedTag>().ForEach( ( Entity entity ) =>
                {
                    float2 targetLocation = new float2( mousePosition.x , mousePosition.z );
                    PathFindingOrders orders = new PathFindingOrders { targetPosition = targetLocation };
                    PostUpdateCommands.AddComponent( entity , orders );
                } );
            }
            else // Player started a formation control
            {
                Entities.WithAll<SelectedTag>().ForEach( ( Entity entity ) =>
                {
                    // Get all selected groups
                    //   we need the order they are selected in
                    //   their positions
                    // Calculate the reserved space for each group
                    //   length of line / number of groups
                    // Foreach (group)
                    //   number of units
                    //   unit spacing

                    float2 targetLocation = new float2( mousePosition.x , mousePosition.z );

                    PathFindingOrders orders = new PathFindingOrders { targetPosition = targetLocation };
                    PostUpdateCommands.AddComponent( entity , orders );
                } );
            }
        }
    }

    private void Click_Select( float3 mousePosition )
    {
        Entities.ForEach( ( Entity entity , ref Translation translation ) =>
        {
            float2 mouseGridPos = new float2( mousePosition.x , mousePosition.z );

            //float distance = math.distance( mouseGridPos , data.position2D );
            //float halfScaleX = data.unitsWide / 2;
            //float halfScaleZ = data.unitsLong / 2;

            float3 lowerLeft = new float3( translation.Value.x - 1 , 0 , translation.Value.z - 1 );
            float3 upperRight = new float3( translation.Value.x + 1 , 0 , translation.Value.z + 1 );

            if ( RectHelper.PointIsWithinRect( mousePosition , lowerLeft , upperRight ) )
            {
                PostUpdateCommands.AddComponent<SelectedTag>( entity );
            }
        } );
    }
    private void Drag_Select( float3 boxStart , float3 boxEnd )
    {
        Entities.ForEach( ( Entity entity , ref Translation translation ) =>
        {
            //float halfScaleX = data.unitsWide / 2;
            //float halfScaleZ = data.unitsLong / 2;

            float3 groupStart = new float3( translation.Value.x - 1 , 0 , translation.Value.z - 1 );
            float3 groupEnd = new float3( translation.Value.x + 1 , 0 , translation.Value.z + 1 );

            if ( RectHelper.RectanglesTouch( boxStart , boxEnd , groupStart , groupEnd ) )
            {
                PostUpdateCommands.AddComponent<SelectedTag>( entity );
            }
        } );
    }
}