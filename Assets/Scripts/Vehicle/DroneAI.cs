using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car.Map;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    private DroneController m_Drone; // the controller we want to use
    private MapManager mapManager;
    private BoxCollider droneCollider;

    private void Start()
    {
        droneCollider = gameObject.GetComponent<BoxCollider>();

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        mapManager = FindObjectOfType<GameManager>().mapManager;
        Vector3 start_pos = mapManager.GetGlobalStartPosition();
        Vector3 goal_pos = mapManager.GetGlobalGoalPosition();

        List<Vector3> my_path = new List<Vector3>();


        // Plan your path here
        Vector3 someLocalPosition = transform.localPosition; // Position of car w.r.p map coordinate origin (not world global)
        // transform.localRotation;  Rotation w.r.p map coordinate origin (not world global)

        // This is how you access information about specific points
        var obstacleMap = mapManager.GetObstacleMap();
        obstacleMap.IsPointTraversable(someLocalPosition);

        // Local to grid and inverse. See other methods for more.
        obstacleMap.grid.LocalToCell(someLocalPosition);

        // This is how you access a traversability grid or gameObjects in each cell.
        Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
        Dictionary<Vector2Int, List<GameObject>> gameObjectsData = obstacleMap.gameGameObjectsPerCell;
        // Easy way to find all position vectors is either "Keys" in above dictionary or:
        foreach (var posThreeDim in obstacleMap.mapBounds.allPositionsWithin)
        {
            Vector2Int gridPos = new Vector2Int(posThreeDim.x, posThreeDim.z);
        }
        // If you need more details, feel free to check out the ObstacleMap class internals.

        // Replace the code below that makes a random path
        // ...
        my_path.Add(start_pos);

        for (int i = 0; i < 3; i++)
        {
            Vector3 waypoint = start_pos + new Vector3(UnityEngine.Random.Range(-50.0f, 50.0f), 0, UnityEngine.Random.Range(-30.0f, 30.0f));
            my_path.Add(waypoint);
        }

        my_path.Add(goal_pos);


        // Plot your path to see if it makes sense
        Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }
    }


    private void FixedUpdate()
    {
        // How to calculate if a physics collider overlaps another.
        var sampleObstacle = mapManager.GetObstacleMap().obstacleObjects[0];
        bool overlapped = Physics.ComputePenetration(
            droneCollider, transform.position, transform.rotation, // Use global position 
            sampleObstacle.GetComponent<MeshCollider>(), // Can take any collider and "project" it using position and rotation vectors.
            sampleObstacle.transform.position,
            sampleObstacle.transform.rotation,
            out var direction, out var distance
        );
        // 'out's give shortest direction and distance to "uncollide" two objects.
        if (overlapped || distance > 0)
        {
            // Means collider inside another   
        }
        // For more details https:docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html
        ///////////////////////////


        // This is how you access information about the terrain from a simulated laser range finder
        RaycastHit hit;
        float maxRange = 50f;
        if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
        {
            Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
            Debug.Log("Did Hit");
        }

        // this is how you control the drone
        m_Drone.Move(0.4f * Mathf.Sin(Time.time * 1.9f), 0.1f);
    }
}