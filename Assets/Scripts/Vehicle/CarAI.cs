using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Linq;
using dubins;
using UnityEngine.UIElements;
using aStar;

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
        public float k_p = 2f;
        public float k_i = 0.1f;
        public float k_d = 0.5f;
        public float nodeDistThreshold = 0.3f;

        Rigidbody my_rigidbody;
        private Vector3 oldTargetPosition;

        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        private List<Vector3> path = new();
        private int currentNodeIdx;
        private float integral = 0f;

        private HybridAStarGenerator pathFinder = null;
        private DubinsGeneratePaths dubinsPathGenerator;
        public bool drawDebug = false;

        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            m_Car = GetComponent<CarController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;
            my_rigidbody = GetComponent<Rigidbody>();

            // TODO: rescale grid to have square shaped grid cells of length=carLength
            // float carLength = carCollider.transform.localScale.z;
            // carLength = 4f;
            // Debug.Log("Global carLength: " + carLength);
            // Vector3 gridScale = mapManager.grid.transform.localScale;
            // mapManager.grid.cellSize = new Vector3(carLength / gridScale.x, carLength / gridScale.y, carLength / gridScale.z);
            // mapManager.Initialize();

            Vector3 localStart = mapManager.localStartPosition;
            Vector3 localGoal = mapManager.localGoalPosition;
            oldTargetPosition = transform.position;
            
            currentNodeIdx = 0;
            pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), m_Car, carCollider);
            List<AStarNode> nodePath = pathFinder.GeneratePath(
                new Vector3(localStart.x, 0.01f, localStart.z),
                new Vector3(localGoal.x, 0.01f, localGoal.z),
                transform.eulerAngles.y);
            
            foreach (var node in nodePath)
            {
                path.Add(node.LocalPosition);
            }

            // Draw unsmoothed path
            Vector3 old_wp = localStart;
            foreach (var wp in path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.magenta, 1000f);
                old_wp = wp;
            }

            // dubinsPathGenerator = new DubinsGeneratePaths();
            // path = GenerateSmoothedPath();

            // // Draw smoothed path
            // old_wp = localStart;
            // foreach (var wp in path)
            // {
            //     Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
            //     old_wp = wp;
            // }
            
        }        

        private List<Vector3> GenerateAStarPath(Vector3 localStart, Vector3 localGoal)
        {
            var roadPath = new List<Vector3>();

            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;

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
            fScore[startCell] = Vector3.Distance(mapManager.grid.CellToLocal(startCell), mapManager.grid.CellToLocal(goalCell));

            while (openSet.Count > 0)
            {
                var currentCell = openSet.Dequeue().Value;

                if (currentCell == goalCell)
                {
                     // Reconstruct the path if the goal is reached
                    while (currentCell != startCell)
                    {
                        Vector3 current = mapManager.grid.GetCellCenterLocal(currentCell);
                        roadPath.Add(new Vector3(current.x, 0, current.y));
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

                    float tmpGScore = gScore[currentCell] + Vector3.Distance(mapManager.grid.CellToLocal(currentCell), mapManager.grid.CellToLocal(neighbor));

                    if (!gScore.ContainsKey(neighbor) || tmpGScore < gScore[neighbor])
                    {
                        gScore[neighbor] = tmpGScore;
                        float hValue = Vector3.Distance(mapManager.grid.CellToLocal(neighbor), mapManager.grid.CellToLocal(goalCell));
                        if (mapData[new Vector2Int(neighbor.x, neighbor.z)] == ObstacleMap.Traversability.Partial) // negative reward for partialy traversable cells
                            hValue += 1f;

                        fScore[neighbor] = gScore[neighbor] + hValue;
                        parentMap[neighbor] = currentCell;

                        if (!openSet.Contains(neighbor))
                        {
                            openSet.Enqueue(fScore[neighbor], neighbor);

                            // Debug
                            // Vector3 neighborLocal = mapManager.grid.CellToLocal(neighbor);
                            // Debug.DrawLine(mapManager.grid.LocalToWorld(localStart), 
                            //     mapManager.grid.LocalToWorld(new Vector3(neighborLocal.x, 0, neighborLocal.y)), 
                            //     Color.white, 1000f);
                        }
                    }
                }
            }

            return roadPath;
        }

        private List<Vector3> GenerateSmoothedPath()
        {
            // Generate smooth path by creating a dubins path between all path nodes 

            var smoothedPath = new List<Vector3>();
            for (int i = 0; i < path.Count-2; ++i)
            {
                Vector3 currentNode = mapManager.grid.LocalToWorld(path[i]);
                Vector3 nextNode = mapManager.grid.LocalToWorld(path[i+1]);
                Vector3 nextNextNode = mapManager.grid.LocalToWorld(path[i+2]);
                
                Vector3 startDir = nextNode - currentNode;
                Vector3 goalDir = nextNextNode - nextNode;

                Quaternion startRotation = Quaternion.LookRotation(startDir);
                Quaternion goalRotation = Quaternion.LookRotation(goalDir);

                // Convert quaternions to Euler angles in degrees
                float startHeading = startRotation.eulerAngles.y * Mathf.Deg2Rad;
                float goalHeading = goalRotation.eulerAngles.y * Mathf.Deg2Rad;

                List<DubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
                    currentNode,
                    startHeading,
                    nextNode,
                    goalHeading);

                if (pathDataList.Count > 0)
                {
                    // TODO: Check for collisions and choose path that does not collide
                    DubinsPath pathData = pathDataList[0];
                    List<Vector3> pathCoordinates = pathData.pathCoordinates;

                    foreach (Vector3 pathCoord in pathCoordinates)
                    {
                        smoothedPath.Add(mapManager.grid.WorldToLocal(pathCoord));
                    }
                }
            }
            smoothedPath.Add(path[path.Count-1]);
            return smoothedPath; // smoothed path with local coordinates
        }

        private IEnumerable<Vector3Int> GetNeighborCells(Vector3Int cell)
        {
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;

            // First four are the non-diagonal neighbor offsets
            int[] xOffsets = { -1, 0, 1, 0, 1, 1, -1, -1 };
            int[] zOffsets = { 0, 1, 0, -1, 1, -1, 1, -1 };

            for (int i = 0; i < xOffsets.Length; i++)
            {
                int x_offset = xOffsets[i];
                int z_offset = zOffsets[i];

                var neighbor = cell;
                neighbor.x += x_offset;
                neighbor.z += z_offset;
                var neighbor2d = new Vector2Int(neighbor.x, neighbor.z);

                if (mapData.ContainsKey(neighbor2d) && mapData[neighbor2d] != ObstacleMap.Traversability.Blocked)
                {
                    yield return neighbor;
                }
            }
        }

        private void FixedUpdate()
        {
            // TODO: Implement backing up of car if stuck (velocity near zero and/or colliding with object)
            if (path.Count == 0)
            {
                return;
            }

            var globalPosition = transform.position;
            Vector3 localPosition = mapManager.grid.WorldToLocal(transform.position);
            
            Vector3 localNextNode = path.ElementAt(currentNodeIdx);
            Vector3 globalNextNode = mapManager.grid.LocalToWorld(localNextNode);


            PidControllTowardsPosition(globalNextNode);
            // If car is at pathNode, update pathNodeEnumerator
            if (currentNodeIdx < path.Count - 1 && Vector3.Distance(mapManager.grid.WorldToLocal(globalPosition), localNextNode) < nodeDistThreshold)
            {
                currentNodeIdx++;
            }

            Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);
        }

        private void PidControllTowardsPosition(Vector3 globalTargetPosition)
        {
            // TODO: add function based on node angle to decide velocity
            Vector3 targetPosition;

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                targetPosition = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else // target is next node in path
            {
                targetPosition = globalTargetPosition;
                target_velocity = (targetPosition - oldTargetPosition) / Time.fixedDeltaTime;
            }

            oldTargetPosition = targetPosition;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 positionError = targetPosition - transform.position;
            Vector3 velocityError = target_velocity - my_rigidbody.velocity;

            Vector3 proportional = k_p * positionError;
            integral += Time.fixedDeltaTime * positionError.magnitude;
            Vector3 integralTerm = k_i * integral * positionError.normalized;
            Vector3 derivativeTerm = k_d * velocityError;

            Vector3 desired_acceleration = proportional + integralTerm + derivativeTerm;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red);
            Debug.DrawLine(oldTargetPosition, oldTargetPosition + my_rigidbody.velocity, Color.magenta);
            Debug.DrawLine(oldTargetPosition, oldTargetPosition + desired_acceleration, Color.black);

            // this is how you control the car
            // Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }

        void OnDrawGizmos()
        {
            if (drawDebug && pathFinder != null && pathFinder.flowField != null)
            {
                foreach (var posEntity in pathFinder.flowField)
                {
                    var position = new Vector3Int(posEntity.Key.x, 1, posEntity.Key.y);

                    var cellToLocal = mapManager.grid.CellToLocal(position);
                    cellToLocal = new Vector3(cellToLocal.x, 1, cellToLocal.y);
                    cellToLocal += mapManager.grid.cellSize / 2;
                    cellToLocal = mapManager.grid.transform.TransformPoint(cellToLocal);

                    var gizmoSize = mapManager.grid.cellSize;
                    gizmoSize.y = 0.005f;
                    gizmoSize.Scale(mapManager.grid.transform.localScale * 0.8f);
                    
                    Gizmos.color = Mathf.CorrelatedColorTemperatureToRGB(1000f + 100f * posEntity.Value);
                    Gizmos.DrawCube(cellToLocal, gizmoSize);
                }
            }
        }
    }

    public class PriorityQueue<TPriority, TValue>
    {
        private readonly List<Node> elements = new();

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

                (elements[childIndex], elements[index]) = (elements[index], elements[childIndex]);
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