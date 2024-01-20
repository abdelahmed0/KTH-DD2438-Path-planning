﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;

        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            m_Car = GetComponent<CarController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;


            // Plan your path here
            Vector3 someLocalPosition = mapManager.grid.WorldToLocal(transform.position); // Position of car w.r.p map coordinate origin (not world global)
            // transform.localRotation;  Rotation w.r.p map coordinate origin (not world global)

            // This is how you access information about specific points
            var obstacleMap = mapManager.GetObstacleMap();
            obstacleMap.IsLocalPointTraversable(someLocalPosition);

            // Local to grid . See other methods for more.
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

            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 goal_pos = mapManager.localGoalPosition;
            
            // TODO: Implement RRT
            // TODO: Consider all obstacle free grid cells instead of only roads
            List<Vector3> my_path = GenerateSimpleRoadPath(start_pos, goal_pos);


            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
                old_wp = wp;
            }
        }

        private List<Vector3> GenerateSimpleRoadPath(Vector3 localStart, Vector3 localGoal)
        {
            List<Vector3> path = new List<Vector3>();

            // Get all road transforms
            List<Transform> roads = new List<Transform>();
            foreach (Transform child in mapManager.grid.transform.Find("Ground 0"))
            {
                if (child.gameObject.name.Contains("road"))
                {
                    roads.Add(child.transform);
                }
            }

            // Find the closest road to the start and goal positions
            Transform startRoad = FindClosestRoad(localStart, roads);
            Transform goalRoad = FindClosestRoad(localGoal, roads);

            // Perform DFS to find a path through the road network
            HashSet<Transform> visited = new HashSet<Transform>();
            Stack<Transform> stack = new Stack<Transform>();
            Dictionary<Transform, Transform> parentMap = new Dictionary<Transform, Transform>();

            stack.Push(startRoad);
            visited.Add(startRoad);

            while (stack.Count > 0)
            {
                Transform currentRoad = stack.Pop();
                // Debug.Log(mapManager.grid.WorldToLocal(currentRoad.position));
                if (currentRoad == goalRoad)
                {
                    // Reconstruct the path if the goal is reached
                    path.Add(localGoal);
                    while (currentRoad != null)
                    {
                        path.Add(mapManager.grid.WorldToLocal(currentRoad.position));
                        currentRoad = parentMap.GetValueOrDefault(currentRoad);
                    }
                    path.Reverse();
                    break;
                }

                foreach (Transform neighbor in GetRoadNeighbors(currentRoad, roads))
                {
                    if (!visited.Contains(neighbor))
                    {
                        stack.Push(neighbor);
                        visited.Add(neighbor);
                        parentMap[neighbor] = currentRoad;
                    }
                }
            }

            return path;
        }

        private Transform FindClosestRoad(Vector3 localPosition, List<Transform> roads)
        {
            Transform closestRoad = null;
            float minDistance = float.MaxValue;

            foreach (Transform road in roads)
            {
                float distance = Vector3.Distance(localPosition, mapManager.grid.WorldToLocal(road.position));
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestRoad = road;
                }
            }

            return closestRoad;
        }

        private IEnumerable<Transform> GetRoadNeighbors(Transform road, List<Transform> roads)
        {
            // TODO: Only connect to free neighbors
            // TODO: Consider different neighboring condition since current depends on length of road pieces
            foreach (Transform otherRoad in roads)
            {
                if (otherRoad != road)
                {
                    float distance = Vector3.Distance(mapManager.grid.WorldToLocal(road.position), mapManager.grid.WorldToLocal(otherRoad.position));
                    if (distance < 2.5f)
                    {
                        yield return otherRoad;
                    }
                }
            }
        }

        private void FixedUpdate()
        {
            // How to calculate if a physics collider overlaps another.
            var exampleObstacle = mapManager.GetObstacleMap().obstacleObjects[0];

            var globalPosition = transform.position;

            bool overlapped = Physics.ComputePenetration(
                carCollider,
                globalPosition,
                transform.rotation, // Use global position 
                exampleObstacle.GetComponent<MeshCollider>(), // Can take any collider and "project" it using position and rotation vectors.
                exampleObstacle.transform.position,
                exampleObstacle.transform.rotation,
                out var direction,
                out var distance
            );
            // 'out's give shortest direction and distance to "uncollide" two objects.
            if (overlapped || distance > 0)
            {
                // Means collider inside another
            }
            // For more details https:docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html
            ///////////////////////////

            // This is how you access information about the terrain from a simulated laser range finder
            // It might be wise to use this for error recovery, but do most of the planning before the race clock starts
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);
             //   Debug.Log("Did Hit");
            }

            Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
            Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);

            // Check and print traversability of currect position
            Vector3 myLocalPosition = mapManager.grid.WorldToLocal(transform.position); // Position of car w.r.p map coordinate origin (not world global)
            var obstacleMap = mapManager.GetObstacleMap();
            Debug.Log(obstacleMap.IsLocalPointTraversable(myLocalPosition));

            // Execute your path here
            // ...

            // this is how you control the car
            m_Car.Move(1f, 1f, 1f, 0f);
        }
    }
}