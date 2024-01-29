
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using UnityStandardAssets.Vehicles.Car;

namespace aStar
{
    public class HybridAStarGenerator
    {
        // Hybrid A-Star implementation as described in paper
        // "Application of Hybrid A* to an Autonomous Mobile Robot for Path Planning in Unstructured Outdoor Environments"
        private const float goalThreshold = 0.4f;
        private readonly float stepDistance;
        private readonly float globalStepDistance;
        private readonly float startingAngleRadians;
        private readonly Grid grid;
        private readonly ObstacleMap obstacleMap;
        private readonly CarController car;
        private readonly BoxCollider carCollider;
        private readonly float carLength;
        private readonly float[] steeringAngles;

        private Vector3 localStart;
        private Vector3 localGoal;

        public HybridAStarGenerator(float startingAngle, Grid grid, ObstacleMap obstacleMap, CarController car, BoxCollider carCollider)
        {
            startingAngleRadians = (startingAngle + 90) * Mathf.Deg2Rad;
            this.grid = grid;
            this.obstacleMap = obstacleMap;
            this.car = car;
            this.carCollider = carCollider;
            this.carLength = grid.WorldToLocal(carCollider.transform.localScale).z;

            // Incorporate cellSize and cellGap to prevent steps from landing in the same cell they started in
            Vector3 cellSize = grid.cellSize;
            Vector3 cellGap = grid.cellGap;
            float cellDiagonal = Mathf.Sqrt(cellSize.x * cellSize.x + cellSize.z * cellSize.z);
            float gapDiagonal = Mathf.Sqrt(cellGap.x * cellGap.x + cellGap.z * cellGap.z);
            stepDistance = cellDiagonal + gapDiagonal + 0.001f;
            Vector3 temp = grid.LocalToWorld(cellSize + cellGap);
            globalStepDistance = Mathf.Sqrt(temp.x * temp.x + temp.z * temp.z);
            Debug.Log("Step distance: " + stepDistance);
            Debug.Log("Global step distance: " + globalStepDistance);
            Debug.Log("Car length" + carLength);

            steeringAngles = new float[] { -car.m_MaximumSteerAngle * Mathf.Deg2Rad, 0f, car.m_MaximumSteerAngle * Mathf.Deg2Rad };
            Debug.Log("Maximum steering angle: " + car.m_MaximumSteerAngle);
        }


        public List<AStarNode> GeneratePath(Vector3 localStart, Vector3 localGoal)
        {
            this.localStart = localStart;
            this.localGoal = localGoal;

            // Perform Hybrid-A* to find a path 
            var openSet = new PriorityQueue();
            var closedSet = new HashSet<AStarNode>();
            // var closedCells = new HashSet<Vector3Int>();

            var startNode = new AStarNode(localStart, startingAngleRadians, null, grid)
            {
                gScore = 0, hScore = Heuristic(localStart)
            };
            openSet.Enqueue(startNode.GetFScore(), startNode);

            int steps = 0;
            while (openSet.Count > 0 && steps < 10000) // TODO: remove, just for debugging
            {
                steps++;
                var currentNode = openSet.Dequeue().Value;
                closedSet.Add(currentNode);
                // closedCells.Add(grid.LocalToCell(currentNode.LocalPosition));

                if (GoalReached(currentNode.LocalPosition, localGoal))
                {
                    // Reconstruct the path if the goal is reached
                    var path = currentNode.BackTrackPath();
                    path.Reverse();
                    return path;
                }

                // Update neighbors
                foreach (AStarNode nextNode in GenerateChildNodes(currentNode))
                {
                    if (closedSet.Contains(nextNode))
                    // if (closedCells.Contains(grid.LocalToCell(nextNode.LocalPosition)))
                    {
                        continue;
                    }

                    Vector3 nextGlobal = grid.LocalToWorld(nextNode.LocalPosition);
                    
                    Vector3Int nextCell = grid.LocalToCell(nextNode.LocalPosition);

                    if (!IsInMapBounds(nextNode.LocalPosition)
                        // || obstacleMap.IsLocalPointTraversable(nextNode.LocalPosition) == ObstacleMap.Traversability.Blocked
                        || Physics.Raycast(grid.LocalToWorld(new Vector3(currentNode.LocalPosition.x, 0.1f, currentNode.LocalPosition.z)),
                            AngleToDirection(currentNode.angle), globalStepDistance)) // FIXME does not recognize blocked blocks
                    {
                        // Debug
                        Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), 
                            nextGlobal, 
                            Color.red, 1000f);
                        closedSet.Add(nextNode);
                        // closedCells.Add(grid.LocalToCell(nextNode.LocalPosition));
                        continue;
                    }
                    // Debug
                    Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), 
                        nextGlobal, 
                        Color.green, 1000f);
                    if (!openSet.UpdateValue(nextNode)) // Enqueue if node was not updated
                    {
                        openSet.Enqueue(nextNode.GetFScore(), nextNode);
                    }
                }
            }

            Debug.Log("No path found");
            return new List<AStarNode>();
        }

        private IEnumerable<AStarNode> GenerateChildNodes(AStarNode parent)
        {
            foreach (float steeringAngle in steeringAngles)
            {
                var nextNode = parent.Copy();

                float turningAngle = SteeringToTurningAngle(steeringAngle);
                Debug.Log("Turning angle: "+ turningAngle * Mathf.Rad2Deg);
                if (Mathf.Abs(turningAngle) > 0.001f)
                {
                    float turningRadius = stepDistance / turningAngle;

                    // Circle center TODO: save and use for path following
                    float cX = parent.LocalPosition.x - Mathf.Sin(parent.angle) * turningRadius;
                    float cZ = parent.LocalPosition.z + Mathf.Cos(parent.angle) * turningRadius;
                    
                    
                    float x = cX + Mathf.Sin(parent.angle + turningAngle) * turningRadius;
                    float z = cZ - Mathf.Cos(parent.angle + turningAngle) * turningRadius;
    
                    nextNode.angle = (nextNode.angle + turningAngle) % (2 * Mathf.PI);
                    Debug.Log("Acc turning angle: " + nextNode.angle * Mathf.Rad2Deg);
                    
                    nextNode.LocalPosition = new Vector3(x, nextNode.LocalPosition.y, z);
                } 
                else
                {
                    nextNode.LocalPosition = new Vector3(
                        nextNode.LocalPosition.x + stepDistance * Mathf.Cos(parent.angle), 
                        nextNode.LocalPosition.y, 
                        nextNode.LocalPosition.z + stepDistance * Mathf.Sin(parent.angle));
                }
                nextNode.parent = parent;
                nextNode.hScore = Heuristic(nextNode.LocalPosition);
                nextNode.gScore += stepDistance; // TODO: fix since driving on a circle not straight line

                yield return nextNode;
                
            }
        }

        private float SteeringToTurningAngle(float steeringAngle)
        {
            return stepDistance / carLength * Mathf.Tan(steeringAngle);
        }

        private float Heuristic(Vector3 localPosition)
        {
            return Vector3.Distance(localPosition, localGoal);
        }

        private bool GoalReached(Vector3 postion, Vector3 goalPosition)
        {
            return Vector3.Distance(postion, goalPosition) < goalThreshold;
        }

        private bool IsInMapBounds(Vector3 localPosition)
        {
            return obstacleMap.mapBounds.Contains(Vector3Int.RoundToInt(localPosition)); 
        }

        public float DirectionToAngle(Vector3 direction)
        {
            return Vector3.Angle(Vector3.zero, direction) * Mathf.Deg2Rad;
            // // Convert quaternions to Euler angles in degrees
            // Quaternion rotation = Quaternion.LookRotation(direction);
            // return rotation.eulerAngles.y * Mathf.Deg2Rad;
        }

        public Vector3 AngleToDirection(float angle)
        {
            // Convert angle in radians to direction
            Quaternion rotation = Quaternion.Euler(0, angle * Mathf.Rad2Deg, 0);
            Vector3 rotatedDirection = rotation * Vector3.forward;
            
            return rotatedDirection; 
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
            // There should only be one node at each position so two nodes are equal if their positions are
            if (obj == null || GetType() != obj.GetType())
            {
                return false;
            }

            AStarNode otherNode = (AStarNode)obj;
            return grid.LocalToCell(LocalPosition).Equals(grid.LocalToCell(otherNode.LocalPosition))
                    && Mathf.Abs(angle-otherNode.angle) < 2.5f; // TODO: export; This is angleResolution / 2 for angleResolution = 5f
        }

        public override int GetHashCode()
        {
            return LocalPosition.GetHashCode();
        }

        public float GetFScore()
        {
            return hScore + gScore;
        }

        public Vector3 GetGlobalPosition(Grid grid)
        {
            return grid.LocalToWorld(LocalPosition);
        }
        
        public void SetGlobalPosition(Grid grid, Vector3 globalPosition)
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

    public class PriorityQueue
    {
        // Min-heap Priority Queue
        private readonly List<Node> elements = new();

        public int Count => elements.Count;

        public void Enqueue(float priority, AStarNode value)
        {
            var newNode = new Node(priority, value);
            elements.Add(newNode);
            int index = elements.Count - 1;

            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (Comparer<float>.Default.Compare(elements[parentIndex].Priority, newNode.Priority) <= 0)
                    break;

                elements[index] = elements[parentIndex];
                index = parentIndex;
            }

            elements[index] = newNode;
        }

        public KeyValuePair<float, AStarNode> Dequeue()
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
                if (rightChildIndex < lastIndex && Comparer<float>.Default.Compare(elements[rightChildIndex].Priority, elements[childIndex].Priority) < 0)
                    childIndex = rightChildIndex;

                if (Comparer<float>.Default.Compare(elements[childIndex].Priority, elements[index].Priority) >= 0)
                    break;

                (elements[childIndex], elements[index]) = (elements[index], elements[childIndex]);
                index = childIndex;
            }

            return new KeyValuePair<float, AStarNode>(frontItem.Priority, frontItem.Value);
        }

        public bool UpdateValue(AStarNode node)
        {
            // Update node and return true if updated, otherwise false
            for (int i = 0; i < elements.Count; i++)
            {
                if (EqualityComparer<AStarNode>.Default.Equals(elements[i].Value, node))
                {
                    if (node.gScore < elements[i].Value.gScore)
                    {
                        // Update gScore and Re-heapify the priority queue
                        elements[i].Value.gScore = node.gScore;
                        Heapify(i);
                    }
                    return true;
                }
            }
            return false;
        }

        private void Heapify(int index)
        {
            while (index > 0)
            {
                int parentIndex = (index - 1) / 2;
                if (Comparer<float>.Default.Compare(elements[parentIndex].Priority, elements[index].Priority) <= 0)
                    break;

                (elements[parentIndex], elements[index]) = (elements[index], elements[parentIndex]);
                index = parentIndex;
            }
        }

        private class Node
        {
            public float Priority { get; }
            public AStarNode Value { get; set; }

            public Node(float priority, AStarNode value)
            {
                Priority = priority;
                Value = value;
            }
        }
    }
}