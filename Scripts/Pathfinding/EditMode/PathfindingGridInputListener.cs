using UnityEngine;
using UnityEditor;

[CustomEditor( typeof( PathfindingGrid ) )]
public class MyCustomEditor : Editor
{
    void OnSceneGUI()
    {
        if ( Application.isEditor )
        {
            Event e = Event.current;
            switch ( e.type )
            {
                case EventType.KeyDown:
                {
                    if ( Event.current.keyCode == ( KeyCode.K ) )
                    {
                        Undo.RecordObject( PathfindingGrid.instance.gameObject , "1" );
                        PathfindingGrid.instance.AddBlockedTile( e.mousePosition );
                    }
                    break;
                }
            }
        }
    }
}
