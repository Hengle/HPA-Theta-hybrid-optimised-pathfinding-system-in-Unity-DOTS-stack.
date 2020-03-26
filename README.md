# HPA-Theta-hybrid-optimised-pathfinding-system-in-Unity-DOTS-stack.

UNITY 2019.3.2f1

Requires: Burst, Jobs, Entities, Mathematics, HybridRenderer, UnityCollections.

This is a simple Unity Scene where you can easily make a grid with obstacles and test out pathfidning. It is multi-threaded and BURSTCOMPILED so it is very fast.

INSTRUCTIONS: 

Very simple. Just drag the Pathfinding Grid object into the scene. Tick the Initialize and Update boxes in the inspector once each. They will not stay ticked. Then, select the grid in the scene and hover your mose over a grid spot and press "K" to add a blocked cell.

Simple ctrl-s (save) and run the game. Move your camera around with WSAD, use QE to rotate lef/right.
Scroll mouse wheel to zoom.
Holde scroll wheel button and move mouse to rotate camera in any direction. 
Double click the ground to zoom to a location. 
Hold L-Shift while moving to speed up.

Select a single unit by clicking on one, or select multiple by dragging over a bunch. Right click somewhere while units are selected to give them the order to move.

DEBUGGING:

To enable/disable visual path markers and pathfinding time check the booleans in the OnUpdate in PathfindingSystem.

USEFUL LINKS:

https://harablog.files.wordpress.com/2009/06/beyondastar.pdf
https://www.gamasutra.com/view/feature/131505/toward_more_realistic_pathfinding.php?print=1
