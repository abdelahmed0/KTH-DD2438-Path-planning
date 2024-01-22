using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Linq;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        public bool driveInCircle = false;
        public float circleRadius = 15f;
        public float circleSpeed = 5f;
        float alpha = 0f;
        public Vector3 circleCenter = Vector3.zero;
        private Vector3 target_velocity;
        public float k_p = 0.1f;
        public float k_d = 0.2f;
        public float nodeDistThreshold = 0.6f;

        Rigidbody my_rigidbody;

        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        private List<Vector3> path;
        private int currentNodeIdx;
        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            m_Car = GetComponent<CarController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;
            my_rigidbody = GetComponent<Rigidbody>();

            Vector3 localStart = mapManager.localStartPosition;
            Vector3 localGoal = mapManager.localGoalPosition;

            Vector3 globalStart = mapManager.grid.LocalToWorld(localStart);
            
            // TODO: Consider all obstacle free grid cells instead of only roads
            // Tree rrtTree = new Tree(globalStart);
            // List<Vector3> rrtPath = GenerateRRTPath(rrtTree.Root, localGoal);
            currentNodeIdx = 0;
            path = GenerateAStarPath(localStart, localGoal);
            
            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = localStart;
            foreach (var wp in path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
                old_wp = wp;
            }
        }
        

        private List<Vector3> GenerateAStarPath(Vector3 localStart, Vector3 localGoal)
        {
            var roadPath = new List<Vector3>();

            Vector3Int startCell = mapManager.grid.LocalToCell(localStart);
            Vector3Int goalCell = mapManager.grid.LocalToCell(localGoal);

            // Perform A* to find a path 
            var openSet = new PriorityQueue<float, Vector3Int>();
            var closedSet = new HashSet<Vector3Int>();
            var parentMap = new Dictionary<Vector3Int, Vector3Int>();
            var gScore = new Dictionary<Vector3Int, float>();
            var fScore = new Dictionary<Vector3Int, float>();

            openSet.Enqueue(0, startCell);
            gScore[startCell] = 0;
            fScore[startCell] = Vector3.Distance(mapManager.grid.CellToWorld(startCell), mapManager.grid.CellToWorld(goalCell));

            while (openSet.Count > 0)
            {
                var currentCell = openSet.Dequeue().Value;
                // Debug.DrawLine(mapManager.grid.CellToWorld(current), mapManager.grid.CellToWorld(startCell), Color.red);

                if (currentCell == goalCell)
                {
                     // Reconstruct the path if the goal is reached
                    while (currentCell != startCell)
                    {
                        Vector3 current = mapManager.grid.CellToLocal(currentCell);
                        roadPath.Add(new Vector3(current.x, 0, current.y)); // counteract grids xzy format
                        currentCell = parentMap[currentCell];
                    }
                    roadPath.Add(localStart);
                    roadPath.Reverse();
                    break;
                }

                closedSet.Add(currentCell);

                foreach (Vector3Int neighbor in GetNeighborCells(currentCell))
                {
                    if (closedSet.Contains(neighbor))
                    {
                        continue;
                    }

                    float tmpGScore = gScore[currentCell] + Vector3.Distance(mapManager.grid.CellToWorld(currentCell), mapManager.grid.CellToWorld(neighbor));

                    if (!gScore.ContainsKey(neighbor) || tmpGScore < gScore[neighbor])
                    {
                        gScore[neighbor] = tmpGScore;
                        fScore[neighbor] = gScore[neighbor] + Vector3.Distance(mapManager.grid.CellToWorld(neighbor), mapManager.grid.CellToWorld(goalCell));
                        parentMap[neighbor] = currentCell;

                        if (!openSet.Contains(neighbor))
                        {
                            openSet.Enqueue(fScore[neighbor], neighbor);
                        }
                    }
                }
            }

            return roadPath;
        }


        private List<Vector3> GenerateDFSPath(Vector3 localStart, Vector3 localGoal)
        {
            var roadPath = new List<Vector3>();

            Vector3Int startCell = mapManager.grid.LocalToCell(localStart);
            Vector3Int goalCell = mapManager.grid.LocalToCell(localGoal);

            // Perform DFS to find a path through the road network
            var visited = new HashSet<Vector3Int>();
            var stack = new Stack<Vector3Int>();
            var parentMap = new Dictionary<Vector3Int, Vector3Int>();

            stack.Push(startCell);
            visited.Add(startCell);

            while (stack.Count > 0)
            {
                Vector3Int currentCell = stack.Pop();

                if (currentCell == goalCell)
                {
                    // Reconstruct the path if the goal is reached
                    while (currentCell != startCell)
                    {
                        Vector3 current = mapManager.grid.CellToLocal(currentCell);
                        roadPath.Add(new Vector3(current.x, current.z, current.y)); // counteract grids xzy format
                        currentCell = parentMap[currentCell];
                    }
                    roadPath.Add(localStart);
                    roadPath.Reverse();
                    break;
                }
                foreach (Vector3Int neighbor in GetNeighborCells(currentCell))
                {
                    if (!visited.Contains(neighbor))
                    {
                        // Debug.DrawLine(mapManager.grid.LocalToWorld(current), mapManager.grid.LocalToWorld(neighbor), Color.red, 1000f);
                        stack.Push(neighbor);
                        visited.Add(neighbor);
                        parentMap[neighbor] = currentCell;
                    }
                }
            }

            return roadPath;
        }

        private IEnumerable<Vector3Int> GetNeighborCells(Vector3Int cell)
        {
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;

            // Define offsets for horizontal and vertical neighbors
            int[] xOffsets = { -1, 0, 1, 0 };
            int[] zOffsets = { 0, 1, 0, -1 };

            for (int i = 0; i < xOffsets.Length; i++)
            {
                int x_offset = xOffsets[i];
                int z_offset = zOffsets[i];

                var neighbor = cell;
                neighbor.x += x_offset;
                neighbor.z += z_offset;
                var neighbor2d = new Vector2Int(neighbor.x, neighbor.z);

                if (mapData.ContainsKey(neighbor2d) && mapData[neighbor2d] == ObstacleMap.Traversability.Free)
                {
                    yield return neighbor;
                }
            }
        }

        // private IEnumerable<Vector3Int> GetNeighborCells(Vector3Int cell)
        // {
        //     // Takes diagonal neighbors into consideration as well
        //     var obstacleMap = mapManager.GetObstacleMap();
        //     Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
            
        //     for (int x_offset = -1; x_offset < 2; ++x_offset)
        //     {
        //         for (int z_offset = -1; z_offset < 2; ++z_offset)
        //         {
        //             Vector3Int neighbor = cell + new Vector3Int(x_offset, 0, z_offset);

        //             Vector2Int neighbor2d = new(neighbor.x, neighbor.z);

        //             if (mapData.ContainsKey(neighbor2d) 
        //                 && mapData[neighbor2d] == ObstacleMap.Traversability.Free)
        //             {
        //                 yield return neighbor;
        //             }
        //         }
        //     }
        // }

        private void FixedUpdate()
        {
            // // How to calculate if a physics collider overlaps another.
            // var exampleObstacle = mapManager.GetObstacleMap().obstacleObjects[0];
            // bool overlapped = Physics.ComputePenetration(
            //     carCollider,
            //     globalPosition,
            //     transform.rotation, // Use global position 
            //     exampleObstacle.GetComponent<MeshCollider>(), // Can take any collider and "project" it using position and rotation vectors.
            //     exampleObstacle.transform.position,
            //     exampleObstacle.transform.rotation,
            //     out var direction,
            //     out var distance
            // );
            // // 'out's give shortest direction and distance to "uncollide" two objects.
            // if (overlapped || distance > 0)
            // {
            //     // Means collider inside another
            // }
            // // For more details https:docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html
            // ///////////////////////////


            var globalPosition = transform.position;
            Vector3 localPosition = mapManager.grid.WorldToLocal(transform.position);
            // var obstacleMap = mapManager.GetObstacleMap();
            // Debug.Log(obstacleMap.IsLocalPointTraversable(localPosition));

            // Execute your path here
            if (path.Count() == 0)
            {
                Debug.LogWarning("Path empty!");
            }
            else
            {
                Vector3 localNextNode = path.ElementAt(currentNodeIdx);
                Vector3 globalNextNode = mapManager.grid.LocalToWorld(localNextNode);

                Debug.DrawLine(globalPosition, globalNextNode, Color.cyan);
                PdControllTowardsPosition(globalNextNode);
                // If car is at pathNode, update pathNodeEnumerator
                if (currentNodeIdx < path.Count() - 1 && Vector3.Distance(mapManager.grid.WorldToLocal(globalPosition), localNextNode) < nodeDistThreshold)
                {
                    currentNodeIdx++;
                }
            }

            Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);
        }

        private void PdControllTowardsPosition(Vector3 globalTargetPosition)
        {
            Vector3 globalPosition = transform.position;
            Vector3 target_position;

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                target_position = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else // target is next node in path
            {
                target_position = globalTargetPosition;
                target_velocity = (target_position - globalPosition) / Time.fixedDeltaTime;
            }

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(globalPosition, globalPosition + my_rigidbody.velocity, Color.magenta);
            Debug.DrawLine(globalPosition, globalPosition + desired_acceleration, Color.black);

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }
    }

    public class PriorityQueue<TPriority, TValue>
    {
        private List<Node> elements = new List<Node>();

        public int Count => elements.Count;

        public void Enqueue(TPriority priority, TValue value)
        {
            var newNode = new Node(priority, value);
            elements.Add(newNode);
            int index = elements.Count - 1;

            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (Comparer<TPriority>.Default.Compare(elements[parentIndex].Priority, newNode.Priority) <= 0)
                    break;

                elements[index] = elements[parentIndex];
                index = parentIndex;
            }

            elements[index] = newNode;
        }

        public KeyValuePair<TPriority, TValue> Dequeue()
        {
            if (elements.Count == 0)
                throw new InvalidOperationException("Queue is empty");

            var frontItem = elements[0];
            int lastIndex = elements.Count - 1;
            elements[0] = elements[lastIndex];
            elements.RemoveAt(lastIndex);

            int index = 0;
            while (true)
            {
                int childIndex = index * 2 + 1;
                if (childIndex >= lastIndex)
                    break;

                int rightChildIndex = childIndex + 1;
                if (rightChildIndex < lastIndex && Comparer<TPriority>.Default.Compare(elements[rightChildIndex].Priority, elements[childIndex].Priority) < 0)
                    childIndex = rightChildIndex;

                if (Comparer<TPriority>.Default.Compare(elements[childIndex].Priority, elements[index].Priority) >= 0)
                    break;

                var tmp = elements[index];
                elements[index] = elements[childIndex];
                elements[childIndex] = tmp;
                index = childIndex;
            }

            return new KeyValuePair<TPriority, TValue>(frontItem.Priority, frontItem.Value);
        }

         public bool Contains(TValue value)
        {
            foreach (var element in elements)
            {
                if (EqualityComparer<TValue>.Default.Equals(element.Value, value))
                {
                    return true;
                }
            }
            return false;
        }

        private class Node
        {
            public TPriority Priority { get; }
            public TValue Value { get; }

            public Node(TPriority priority, TValue value)
            {
                Priority = priority;
                Value = value;
            }
        }
    }

}