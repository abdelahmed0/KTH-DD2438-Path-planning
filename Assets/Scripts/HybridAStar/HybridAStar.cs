
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using UnityStandardAssets.Vehicles.Car;
using UnityEngine.UIElements;
using UnityEditorInternal;

namespace aStar
{
    public class HybridAStarGenerator
    {
        // Hybrid A-Star implementation as described in paper
        // "Application of Hybrid A* to an Autonomous Mobile Robot for Path Planning in Unstructured Outdoor Environments"

        // Resolution of 10 corresponds to 360/10=36 possible angles per cell
        public static float angleResolution = 10f;
        private const float goalThreshold = 1.1f;
        private const float colliderResizeFactor = 1f;
        private const int maxSteps = 100000;

        private readonly float stepDistance;
        private readonly float globalStepDistance;
        private readonly Grid grid;
        private readonly ObstacleMap obstacleMap;
        private readonly BoxCollider carCollider;
        private readonly float carLength;
        private readonly float[] steeringAngles;

        private Vector3 localStart;
        private Vector3 localGoal;

        public Dictionary<Vector2Int, float> flowField = null;

        public HybridAStarGenerator(Grid grid, ObstacleMap obstacleMap, float maxSteeringAngle, BoxCollider carCollider)
        {
            this.grid = grid;
            this.obstacleMap = obstacleMap;
            this.carCollider = carCollider;
            carLength = grid.WorldToLocal(carCollider.transform.localScale).z;
            Debug.Log("Collider size: " + grid.WorldToLocal(carCollider.size));
            Debug.Log("Map bounds: " + obstacleMap.mapBounds);

            // Incorporate cellSize and cellGap to prevent steps from landing in the same cell they started in
            Vector3 cellSize = grid.cellSize;
            Debug.Log("Cellsize: " + cellSize);
            Vector3 cellGap = grid.cellGap;
            float cellDiagonal = Mathf.Sqrt(cellSize.x * cellSize.x + cellSize.y * cellSize.y);
            float gapDiagonal = Mathf.Sqrt(cellGap.x * cellGap.x + cellGap.y * cellGap.y);
            stepDistance = cellDiagonal + gapDiagonal + 0.001f;
            Vector3 temp = grid.LocalToWorld(cellSize + cellGap);
            globalStepDistance = Mathf.Sqrt(temp.x * temp.x + temp.y * temp.y);
            Debug.Log("Step distance: " + stepDistance);
            Debug.Log("Global step distance: " + globalStepDistance);
            Debug.Log("Car length" + carLength);

            // steeringAngles = new float[] { -maxSteeringAngle * Mathf.Deg2Rad, -maxSteeringAngle / 2f * Mathf.Deg2Rad, 0f, maxSteeringAngle / 2f * Mathf.Deg2Rad,  maxSteeringAngle * Mathf.Deg2Rad };
            steeringAngles = new float[] { -maxSteeringAngle * Mathf.Deg2Rad, 0f, maxSteeringAngle * Mathf.Deg2Rad };

            Debug.Log("Maximum steering angle: " + maxSteeringAngle);
        }


        public List<AStarNode> GeneratePath(Vector3 localStart, Vector3 localGoal, float startingAngle)
        {
            this.localStart = localStart;
            this.localGoal = localGoal;
            float startingAngleRadians = (startingAngle + 90) * Mathf.Deg2Rad; // TODO: understand why starting angle is rotated

            CalculateFlowField();

            // Perform Hybrid-A* to find a path 
            var openSet = new PriorityQueue<float, AStarNode>();
            HashSet<AStarNode> closedSet = new HashSet<AStarNode>();

            var startNode = new AStarNode(localStart, startingAngleRadians, null, grid)
            {
                gScore = 0, hScore = Heuristic(grid.LocalToWorld(localStart))
            };
            openSet.Enqueue(startNode.GetFScore(), startNode);

            var backwardsNode = startNode.Copy();
            backwardsNode.angle = (startingAngle - 90) * Mathf.Deg2Rad;
            openSet.Enqueue(startNode.GetFScore(), backwardsNode);

            int steps = 0;
            while (openSet.Count > 0 && steps < maxSteps)
            {
                steps++;
                var currentNode = openSet.Dequeue().Value;
                closedSet.Add(currentNode);

                if (GoalReached(currentNode.LocalPosition))
                {
                    var goalNode = currentNode.Copy();
                    goalNode.parent = currentNode;
                    goalNode.LocalPosition = localGoal;
                    // Reconstruct the path if the goal is reached
                    var path = goalNode.BackTrackPath();
                    path.Reverse();
                    return path;
                }

                // Update neighbors
                foreach (AStarNode nextNode in GenerateChildNodes(currentNode))
                {
                    if (closedSet.Contains(nextNode)) // TODO if gScore better, then replace node in closedSet
                    {
                        continue;
                    }

                    Vector3 nextGlobal = grid.LocalToWorld(nextNode.LocalPosition);

                    if (!IsReachable(currentNode, nextNode))
                    {
                        // Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), 
                        //     nextGlobal, 
                        //     Color.red, 1000f);

                        closedSet.Add(nextNode);
                        continue;
                    }
                    Color hColor = Mathf.CorrelatedColorTemperatureToRGB(1000f + 100f * nextNode.hScore);
                    Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), 
                        nextGlobal, 
                        hColor, 1000f);

                    openSet.Enqueue(nextNode.GetFScore(), nextNode); // Enqueue if node was not updated
                }
            }

            Debug.LogWarning("No path found in " + steps + " steps");
            return new List<AStarNode>();
        }

        private IEnumerable<AStarNode> GenerateChildNodes(AStarNode parent)
        {
            foreach (float steeringAngle in steeringAngles)
            {
                var nextNode = parent.Copy();
                nextNode.parent = parent;

                float turningAngle = SteeringToTurningAngle(steeringAngle);
                // Debug.Log("Turning angle: "+ turningAngle * Mathf.Rad2Deg);
                if (Mathf.Abs(turningAngle) > 0.001f)
                {
                    float turningRadius = globalStepDistance / turningAngle;

                    float cX = parent.GetGlobalPosition().x - Mathf.Sin(parent.angle) * turningRadius;
                    float cZ = parent.GetGlobalPosition().z + Mathf.Cos(parent.angle) * turningRadius;
                    
                    float x = cX + Mathf.Sin(parent.angle + turningAngle) * turningRadius;
                    float z = cZ - Mathf.Cos(parent.angle + turningAngle) * turningRadius;
    
                    nextNode.angle = (nextNode.angle + turningAngle) % (2 * Mathf.PI);
                    // Debug.Log("Acc turning angle: " + nextNode.angle * Mathf.Rad2Deg);
                    
                    nextNode.SetGlobalPosition(new Vector3(x, nextNode.GetGlobalPosition().y, z));
                } 
                else
                {
                    nextNode.SetGlobalPosition(nextNode.GetGlobalPosition() + new Vector3(
                        globalStepDistance * Mathf.Cos(parent.angle), 
                        0f, 
                        globalStepDistance * Mathf.Sin(parent.angle)));
                }
                nextNode.hScore = Heuristic(nextNode.GetGlobalPosition());
                nextNode.gScore += stepDistance;

                yield return nextNode;
                
            }
        }

        private bool IsReachable(AStarNode current, AStarNode next)
        { 
            var currentGlobal = current.GetGlobalPosition();
            var nextGlobal = next.GetGlobalPosition();
            var cell = grid.WorldToCell(nextGlobal);
            
            // Check if outside of map
            var cell2d = new Vector2Int(cell.x, cell.y);
            if (!obstacleMap.traversabilityPerCell.ContainsKey(cell2d))
                return false;
                                        
            // Node in blocked cell
            var nextCell = grid.LocalToCell(next.LocalPosition);
            if (obstacleMap.traversabilityPerCell[new Vector2Int(nextCell.x, nextCell.y)] == ObstacleMap.Traversability.Blocked)
                return false;
            
            // Check if path to node is blocked
            Vector3 direction = (nextGlobal - currentGlobal).normalized;
            var orientation = Quaternion.FromToRotation(Vector3.forward, direction);

            bool hit = Physics.BoxCast(currentGlobal - carCollider.transform.localScale.z * direction,
                                        colliderResizeFactor * carCollider.transform.localScale / 2f,
                                        direction, 
                                        out var hitInfo,
                                        orientation,
                                        globalStepDistance);
            // if (hit)
            //     ExtDebug.DrawBoxCastOnHit(currentGlobal - carCollider.transform.localScale.z * direction,
            //                             colliderResizeFactor * carCollider.transform.localScale / 2f,
            //                             direction, 
            //                             orientation,
            //                             hitInfo.distance,
            //                             Color.blue);
            
            return !hit;
    }

        private float SteeringToTurningAngle(float steeringAngle)
        {
            return globalStepDistance / carCollider.transform.localScale.z * Mathf.Tan(steeringAngle);
        }

        private bool GoalReached(Vector3 localPosition)
        {
            Vector3Int cell = grid.LocalToCell(localPosition);
            Vector2Int cell2d = new Vector2Int(cell.x, cell.y);
            if (!flowField.ContainsKey(cell2d))
                return flowField[cell2d] < goalThreshold;
            return Vector2.Distance(new Vector2(localPosition.x, localPosition.z), new Vector2(localGoal.x, localGoal.z)) < goalThreshold;
        }

        private float Heuristic(Vector3 globalPosition)
        {
            Vector3Int cell = grid.WorldToCell(globalPosition);
            Vector2Int cell2d = new Vector2Int(cell.x, cell.y);
            return flowField[cell2d];
        }

        void CalculateFlowField()
        {
            flowField = new Dictionary<Vector2Int, float>(obstacleMap.traversabilityPerCell.Count);
            
            foreach (Vector2Int key in obstacleMap.traversabilityPerCell.Keys)
            {
                flowField.Add(key, int.MaxValue);
            }

            var goalCell = new Vector2Int(grid.LocalToCell(localGoal).x, grid.LocalToCell(localGoal).y); 

            var openSet = new Queue<Vector2Int>();
            openSet.Enqueue(goalCell);
            flowField[goalCell] = 0;

            Vector2Int[] neighbors = new Vector2Int[]
            {
                Vector2Int.down, Vector2Int.up, Vector2Int.left, Vector2Int.right,
                new (-1, -1), new(-1, 1), new(1, -1), new(1, 1)
            };
            float[] dist = new float[]
            {
                stepDistance, stepDistance, stepDistance, stepDistance,
                stepDistance, stepDistance, stepDistance, stepDistance
            };

            // Perform a breadth-first search to calculate the flowfield
            while (openSet.Count > 0)
            {
                Vector2Int currentCell = openSet.Dequeue();

                for (int i = 0; i < neighbors.Length; ++i)
                {
                    Vector2Int offset = neighbors[i];
                    Vector2Int neighborCell = currentCell + offset;

                    // Check if the neighbor is within bounds
                    if (IsCellValid(neighborCell) && flowField[neighborCell] == int.MaxValue)
                    {
                        // Add the neighbor to the open set and update its cost
                        openSet.Enqueue(neighborCell);
                        flowField[neighborCell] = flowField[currentCell] + dist[i];
                    }
                }
            }
        }

        bool IsCellValid(Vector2Int cell)
        {
            return obstacleMap.traversabilityPerCell.ContainsKey(cell) 
                    && obstacleMap.traversabilityPerCell[cell] != ObstacleMap.Traversability.Blocked;
        }
    }


    public class AStarNode
    {
        public Vector3 LocalPosition { get; set; }
        public float angle; // in radians
        public AStarNode parent;
        public float gScore;
        public float hScore;

        private readonly Grid grid;

        public AStarNode(Vector3 localPosition, float angle, AStarNode parent, Grid grid)
        {
            LocalPosition = localPosition;
            this.parent = parent;
            this.angle = angle;
            this.grid = grid;
        }

        public override bool Equals(object obj)
        {
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }

            AStarNode otherNode = (AStarNode)obj;
            int thisRoundedAngle = Mathf.RoundToInt(angle * Mathf.Rad2Deg / HybridAStarGenerator.angleResolution);
            int otherRoundedAngle = Mathf.RoundToInt(otherNode.angle * Mathf.Rad2Deg / HybridAStarGenerator.angleResolution);
            return grid.LocalToCell(LocalPosition).Equals(grid.LocalToCell(otherNode.LocalPosition))
                    && thisRoundedAngle == otherRoundedAngle;
        }

        public override int GetHashCode()
        {
            float roundedAngle = Mathf.RoundToInt(angle * Mathf.Rad2Deg / HybridAStarGenerator.angleResolution);
            return (grid.LocalToCell(LocalPosition).ToString() + roundedAngle.ToString()).GetHashCode();
        }

        public float GetFScore()
        {
            return hScore + gScore;
        }

        public Vector3 GetGlobalPosition()
        {
            return grid.LocalToWorld(LocalPosition);
        }
        
        public void SetGlobalPosition(Vector3 globalPosition)
        {
            LocalPosition = grid.WorldToLocal(globalPosition);
        }

        public AStarNode Copy()
        {
            return new AStarNode(LocalPosition, angle, parent, grid) { gScore = gScore, hScore = hScore };
        }

        public List<AStarNode> BackTrackPath()
        {
            var nodePath = new List<AStarNode>();

            AStarNode current = this;
            while (current != null)
            {
                nodePath.Add(current);
                current = current.parent;
            }
            return nodePath;
        }
    }


    public class PriorityQueue<TPriority, TValue>
    {
        // Min-heap Priority Queue
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

        public bool UpdateValue(TValue node, Func<TValue, TValue, bool> comparisonCheck)
        {
            // Update node and return true if updated, otherwise false
            for (int i = 0; i < elements.Count; i++)
            {
                if (EqualityComparer<TValue>.Default.Equals(elements[i].Value, node))
                {
                    if (comparisonCheck(node, elements[i].Value))
                    {
                        // Update Value and Re-heapify the priority queue
                        elements[i].Value = node;
                        Heapify(i);
                        return true;
                    }
                }
            }
            return false;
        }

        private void Heapify(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (Comparer<TPriority>.Default.Compare(elements[parentIndex].Priority, elements[index].Priority) <= 0)
                    break;

                (elements[parentIndex], elements[index]) = (elements[index], elements[parentIndex]);
                index = parentIndex;
            }
        }

        private class Node
        {
            public TPriority Priority { get; }
            public TValue Value { get; set; }

            public Node(TPriority priority, TValue value)
            {
                Priority = priority;
                Value = value;
            }
        }
    }

}