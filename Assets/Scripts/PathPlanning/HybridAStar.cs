
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using UnityStandardAssets.Vehicles.Car;
using UnityEngine.UIElements;
using UnityEditorInternal;
using System.Linq;

namespace aStar
{
    public class HybridAStarGenerator
    {
        // Hybrid A-Star implementation as described in paper
        // "Application of Hybrid A* to an Autonomous Mobile Robot for Path Planning in Unstructured Outdoor Environments"

        // Resolution of 10 corresponds to 360/10=36 possible angles per cell
        public static float AngleResolution = 10f;
        private const float GoalThreshold = 0.5f;
        private const int maxSteps = 300000;

        private readonly float colliderResizeFactor;
        private readonly float stepDistance;
        private readonly float globalStepDistance;
        private readonly Grid grid;
        private readonly ObstacleMap obstacleMap;
        private readonly BoxCollider collider;
        private readonly float vehicleLength;
        private readonly bool fixedOrientation;
        private readonly float maxSteeringAngle;
        private readonly bool allowReversing = false;
        private readonly float backwardsPenalty;
        private float[] steeringAngles;

        private Vector3 localStart;
        private Vector3 localGoal;

        public Dictionary<Vector2Int, float> flowField = null;

        public HybridAStarGenerator(Grid grid, ObstacleMap obstacleMap, float maxSteeringAngle, BoxCollider collider, float colliderResizeFactor, bool fixedOrientation, bool allowReversing, float backwardsPenalty)
        {
            this.grid = grid;
            this.obstacleMap = obstacleMap;
            this.collider = collider;
            this.colliderResizeFactor = colliderResizeFactor;
            vehicleLength = grid.WorldToLocal(collider.transform.localScale).z;
            Debug.Log("Collider size: " + grid.WorldToLocal(collider.size));
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
            Debug.Log("Car length" + vehicleLength);

            this.maxSteeringAngle = maxSteeringAngle;
            this.fixedOrientation = fixedOrientation;
            this.allowReversing = allowReversing;
            this.backwardsPenalty = backwardsPenalty;
        }


        public List<AStarNode> GeneratePath(Vector3 localStart, Vector3 localGoal, float startingAngle, int numberSteeringAngles)
        {
            this.localStart = localStart;
            this.localGoal = localGoal;
            steeringAngles = new float[numberSteeringAngles];
            for (int i = 0; i < numberSteeringAngles; ++i)
            {
                if (i == numberSteeringAngles / 2)
                    steeringAngles[i] = 0f;
                else 
                    steeringAngles[i] = Mathf.Deg2Rad * maxSteeringAngle / (i - (numberSteeringAngles / 2));
            }

            CalculateFlowField();

            // Perform Hybrid-A* to find a path 
            var openSet = new PriorityQueue<float, AStarNode>();
            HashSet<AStarNode> closedSet = new HashSet<AStarNode>();

            float startingAngleRadians = (startingAngle + 90) * Mathf.Deg2Rad; 
            var startNode = new AStarNode(localStart, startingAngleRadians, null, grid)
            {
                gScore = 0, hScore = Heuristic(grid.LocalToWorld(localStart))
            };
            openSet.Enqueue(startNode.GetFScore(), startNode);

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
                    if (closedSet.Contains(nextNode)) 
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
                    // Color hColor = Mathf.CorrelatedColorTemperatureToRGB(1000f + 100f * nextNode.hScore);
                    // Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), 
                    //     nextGlobal, 
                    //     hColor, 1000f);

                    // if (nextNode.isBackwards)
                    //     Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), nextGlobal, Color.yellow, 1000f);
                    // else
                    //     Debug.DrawLine(grid.LocalToWorld(currentNode.LocalPosition), nextGlobal, Color.green, 1000f);

                    // TODO if gScore better, then replace node in closedSet
                    openSet.Enqueue(nextNode.GetFScore(), nextNode); // Enqueue if node was not updated
                }
            }

            Debug.LogWarning("No path found in " + steps + " steps");
            return new List<AStarNode>();
        }

        private List<AStarNode> GenerateChildNodes(AStarNode parent)
        {
            List<AStarNode> children = new();

            var parentGlobal = parent.GetGlobalPosition();
            foreach (float steeringAngle in steeringAngles)
            {
                // 1 - reverse
                int reversing = allowReversing ? 1 : 0;
                for (int i = 0; i <= reversing; ++i)
                {
                    var nextNode = parent.Copy();
                    nextNode.parent = parent;

                    nextNode.isBackwards = i == 1;
                    float reverseFactor = nextNode.isBackwards ? -1f : 1f;

                    float turningAngle = SteeringToTurningAngle(steeringAngle);

                    // Debug.Log("Turning angle: "+ turningAngle * Mathf.Rad2Deg);
                    if (Mathf.Abs(turningAngle) > 0.001f)
                    {
                        float turningRadius = globalStepDistance / turningAngle;

                        float cX = parentGlobal.x - Mathf.Sin(parent.angle) * turningRadius;
                        float cZ = parentGlobal.z + Mathf.Cos(parent.angle) * turningRadius;
                        
                        float x = cX + Mathf.Sin(parent.angle + reverseFactor * turningAngle) * turningRadius;
                        float z = cZ - Mathf.Cos(parent.angle + reverseFactor * turningAngle) * turningRadius;
        
                        nextNode.angle = (parent.angle + reverseFactor * turningAngle) % (2 * Mathf.PI);
                        
                        nextNode.SetGlobalPosition(new Vector3(x, nextNode.GetGlobalPosition().y, z));
                    }
                    else
                    {
                        nextNode.SetGlobalPosition(nextNode.GetGlobalPosition() + new Vector3(
                            reverseFactor * globalStepDistance * Mathf.Cos(parent.angle), 
                            0f,
                            reverseFactor * globalStepDistance * Mathf.Sin(parent.angle)));

                    }
                    nextNode.hScore = Heuristic(nextNode.GetGlobalPosition()) +  Mathf.Abs(parent.angle - nextNode.angle) / 2f * Mathf.PI;;

                    if (nextNode.isBackwards)
                        nextNode.hScore += backwardsPenalty;

                    nextNode.gScore += stepDistance;
                    
                    children.Add(nextNode);
                }
            }
            return children;
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
            var orientation = fixedOrientation ? Quaternion.Euler(Vector3.forward) : Quaternion.FromToRotation(Vector3.forward, direction);

            bool hit = Physics.BoxCast(currentGlobal - collider.transform.localScale.z * direction,// * colliderResizeFactor,
                                        colliderResizeFactor * collider.transform.localScale / 2f,
                                        direction, 
                                        out var hitInfo,
                                        orientation,
                                        globalStepDistance + collider.transform.localScale.z);// * colliderResizeFactor);
            // if (hit)
            //     ExtDebug.DrawBoxCastOnHit(currentGlobal - collider.transform.localScale.z * direction,// - collider.transform.localScale.z * direction * colliderResizeFactor,
            //                             colliderResizeFactor * collider.transform.localScale / 2f,
            //                             direction, 
            //                             orientation,
            //                             hitInfo.distance,
            //                             Color.blue);
            
            return !hit;
    }

        private float SteeringToTurningAngle(float steeringAngle)
        {
            return globalStepDistance / collider.transform.localScale.z * Mathf.Tan(steeringAngle);
        }

        private bool GoalReached(Vector3 localPosition)
        {
            Vector3Int cell = grid.LocalToCell(localPosition);
            Vector2Int cell2d = new Vector2Int(cell.x, cell.y);
            if (!flowField.ContainsKey(cell2d))
                return flowField[cell2d] < GoalThreshold;
            return Vector2.Distance(new Vector2(localPosition.x, localPosition.z), new Vector2(localGoal.x, localGoal.z)) < GoalThreshold;
        }

        private float Heuristic(Vector3 globalPosition)
        {
            Vector3Int cell = grid.WorldToCell(globalPosition);
            Vector2Int cell2d = new Vector2Int(cell.x, cell.y);
            return flowField[cell2d];
        }

        private void CalculateFlowField()
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
                Mathf.Sqrt(2f * stepDistance * stepDistance), Mathf.Sqrt(2f * stepDistance * stepDistance), Mathf.Sqrt(2f * stepDistance * stepDistance), Mathf.Sqrt(2f * stepDistance * stepDistance)
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

        private bool IsCellValid(Vector2Int cell)
        {
            return obstacleMap.traversabilityPerCell.ContainsKey(cell) 
                    && obstacleMap.traversabilityPerCell[cell] != ObstacleMap.Traversability.Blocked;
        }

        public List<AStarNode> SmoothPath(List<AStarNode> path)
        {
            if (path == null || path.Count == 0)
                return new List<AStarNode>();

            for (int i = 0; i < path.Count-1; ++i)
            {
                AStarNode parent = path[i];
                AStarNode next = path[i+1];
                Vector3 parentGlobal = parent.GetGlobalPosition();

                float turningAngle = next.angle - parent.angle;

                var last = parent;  
                if (Mathf.Abs(turningAngle * Mathf.Rad2Deg) > 0.1f)
                {
                    float turningRadius = globalStepDistance / turningAngle;

                    float cX = parentGlobal.x - Mathf.Sin(parent.angle) * turningRadius;
                    float cZ = parentGlobal.z + Mathf.Cos(parent.angle) * turningRadius;
                    
                    // Calculate the number of intermediate points based on the angle difference
                    int numPoints = Mathf.CeilToInt(Mathf.Abs(turningAngle) * Mathf.Rad2Deg / 5f);
                    // Debug.Log("num points: " + numPoints);
                    for (int j = 1; j < numPoints; j++)
                    {
                        float t = j / (float)numPoints;
                        var intermediate = parent.Copy();
                        intermediate.parent = last;
                        float intermediateAngle = Mathf.LerpAngle(0f, turningAngle * Mathf.Rad2Deg, t) * Mathf.Deg2Rad;

                        float x = cX + Mathf.Sin(parent.angle + intermediateAngle) * turningRadius;
                        float z = cZ - Mathf.Cos(parent.angle + intermediateAngle) * turningRadius;
                        intermediate.angle = (intermediate.angle + turningAngle) % (2 * Mathf.PI);
                        intermediate.SetGlobalPosition(new Vector3(x, intermediate.GetGlobalPosition().y, z));
                        intermediate.hScore = Heuristic(intermediate.GetGlobalPosition());
                        intermediate.gScore += stepDistance;

                        last = intermediate;
                    }

                }
                next.parent = last;
            }

            List<AStarNode> smoothed = path[^1].BackTrackPath();
            smoothed.Reverse();
            return smoothed;
        }
    }


    public class AStarNode
    {
        public Vector3 LocalPosition { get; set; }
        public float angle; // in radians
        public AStarNode parent;
        public float gScore;
        public float hScore;
        public bool isBackwards = false;

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
            int thisRoundedAngle = Mathf.RoundToInt(angle * Mathf.Rad2Deg / HybridAStarGenerator.AngleResolution);
            int otherRoundedAngle = Mathf.RoundToInt(otherNode.angle * Mathf.Rad2Deg / HybridAStarGenerator.AngleResolution);
            return grid.LocalToCell(LocalPosition).Equals(grid.LocalToCell(otherNode.LocalPosition))
                    && thisRoundedAngle == otherRoundedAngle;
        }

        public override int GetHashCode()
        {
            float roundedAngle = Mathf.RoundToInt(angle * Mathf.Rad2Deg / HybridAStarGenerator.AngleResolution);
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
            return new AStarNode(LocalPosition, angle, parent, grid) { gScore = gScore, hScore = hScore, isBackwards = isBackwards };
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



























    public class HybridANode
    {
        public Vector3 position;
        public HybridANode parent;
        public float theta;
        public float velocity;
        public float g;
        public float h;
        public float f;

        public HybridANode(Vector3 position, float theta, HybridANode parent, float g, float h)
        {
            this.position = position;
            this.parent = parent;
            this.theta = theta;
            this.g = g;
            this.h = h;
            this.f = g + h;
        }
    }

    public class HybridAStar
    {
        public Vector3 start;
        public Vector3 goal;
        public Dictionary<Vector2Int, ObstacleMap.Traversability> mapData;
        public float initialAngle;
        public List<Vector3> path;
        public List<float> dx = new List<float> { 0.7068582f, 0.705224f, 0.705224f };
        public List<float> dz = new List<float> { 0, -0.0415893f, 0.0415893f };
        public List<float> dt = new List<float> { 0, 0.1178097f, -0.1178097f };
        public HybridAStar(Vector3 start, Vector3 goal, Dictionary<Vector2Int, ObstacleMap.Traversability> mapData, float initialAngle)
        {
            this.start = start;
            this.goal = goal;
            this.mapData = mapData;
            this.initialAngle = initialAngle;
            this.path = new List<Vector3>();
        }

        public List<Vector3> PlanPath()
        {
            // initialize open and closed list
            List<HybridANode> openList = new List<HybridANode>();
            List<HybridANode> closedList = new List<HybridANode>();

            // store all nodes in a dictionary
            Dictionary<Vector3, HybridANode> nodeDict = new Dictionary<Vector3, HybridANode>();
            foreach (Vector2Int pos in mapData.Keys)
            {
                if (mapData[pos] == ObstacleMap.Traversability.Blocked)
                {
                    continue;
                }
                Vector3 pos3d = new Vector3();

                pos3d = new Vector3(pos.x, pos.y, 0);
                HybridANode node = new HybridANode(pos3d, 0, null, 0, Vector3.Distance(pos3d, goal));
                nodeDict.Add(pos3d, node);

                pos3d = new Vector3(pos.x, pos.y, Mathf.PI);
                node = new HybridANode(pos3d, Mathf.PI, null, 0, Vector3.Distance(pos3d, goal));
                nodeDict.Add(pos3d, node);
                for (int i = 1; i < 18; i++)
                {
                    pos3d = new Vector3(pos.x, pos.y, i * Mathf.PI / 18);
                    node = new HybridANode(pos3d, i * Mathf.PI / 18, null, 0, Vector3.Distance(pos3d, goal));
                    nodeDict.Add(pos3d, node);

                    pos3d = new Vector3(pos.x, pos.y, -i * Mathf.PI / 18);
                    node = new HybridANode(pos3d, -i * Mathf.PI / 18, null, 0, Vector3.Distance(pos3d, goal));
                    nodeDict.Add(pos3d, node);
                }



            }
            HybridANode start_node = nodeDict[LocToGrid(new Vector3((int)this.start.x, (int)this.start.z, this.initialAngle))];
            openList.Add(start_node);

            while (openList.Count > 0)
            {
                // 3. sort open list by f cost primarily and h cost secondarily
                openList.OrderBy(n => n.f).ThenBy(n => n.h).ToList();
                HybridANode current_node = openList[0];

                // 4. if current node is goal node, return path
                if (Vector3.Distance(current_node.position, goal) < 2f)
                {
                    this.path.Add(goal);
                    while (current_node.parent != null)
                    {
                        this.path.Add(current_node.position);
                        current_node = current_node.parent;
                    }
                    this.path.Reverse();
                    break;
                }

                // 5. remove current node from open list and add to closed list
                openList.Remove(current_node);
                closedList.Add(current_node);

                // 6. for each neighbor of current node
                for (int i = 0; i < 3; i++)
                {
                    Vector3 neighbour = GetNextPos(current_node.position, i);
                    neighbour = LocToGrid(neighbour);
                    // the position is Blocked
                    if (!nodeDict.ContainsKey(neighbour))
                    {
                        continue;
                    }
                    if (closedList.Any(node => node.position == neighbour))
                    {
                        continue;
                    }
                    HybridANode neighborNode = nodeDict[neighbour];
                    float new_g = current_node.g + Vector3.Distance(current_node.position, neighborNode.position);
                    Vector3 current_idx = LocToGrid(new Vector3(current_node.position.x, current_node.position.z, current_node.theta));
                    if (!openList.Any(node => (node.position == neighborNode.position && node.theta == neighborNode.theta)) || new_g < nodeDict[neighbour].g)
                    {
                        neighborNode.h = Vector3.Distance(neighborNode.position, goal);
                        neighborNode.g = new_g;
                        neighborNode.f = neighborNode.g + neighborNode.h;
                        neighborNode.parent = current_node;
                        if (!openList.Any(node => (node.position == neighborNode.position && node.theta == neighborNode.theta)))
                        {
                            openList.Add(neighborNode);
                        }
                    }

                }
            }
            return this.path;
        }

        public Vector3 GetNextPos(Vector3 current_pos, int i)
        {
            float x = current_pos.x + dx[i] * Mathf.Cos(current_pos.y) - dz[i] * Mathf.Sin(current_pos.y);
            float y = current_pos.y + dx[i] * Mathf.Sin(current_pos.y) + dz[i] * Mathf.Cos(current_pos.y);
            float theta = NormalRad(current_pos.z + dt[i]);
            return new Vector3(x, y, theta);
        }

        public float NormalRad(float rad)
        {
            if (rad > Mathf.PI)
            {
                rad = rad - 2 * Mathf.PI;
            }
            else if (rad <= -Mathf.PI)
            {
                rad = rad + 2 * Mathf.PI;
            }
            return rad;
        }

        public Vector3 LocToGrid(Vector3 sortkey)
        {
            Vector3 gridPos = new Vector3((int)sortkey.x, (int)sortkey.y, 0);
            float rad = sortkey.z;
            if (rad > 0)
            {
                int i = Mathf.RoundToInt(rad / (Mathf.PI / 18));
                gridPos.z = i * Mathf.PI / 18;
            }
            else if (rad < 0)
            {
                int i = Mathf.RoundToInt(rad / (Mathf.PI / 18));
                gridPos.z = -i * Mathf.PI / 18;
            }
            else
            {
                gridPos.z = 0;
            }
            return gridPos;
        }
    }

    public class VoronoiDiagram : MonoBehaviour
    {
        public Vector2Int dim;

        public List<Vector3> sites;

        public float heightAboveGround = 10f;
        /*
        private void Start()
        {
            List<Vector3> sitesCopy = new List<Vector3>(sites);
            Debug.Log(CreateVoronoiDiagram(sitesCopy));
            Texture2D diagramTexture = CreateVoronoiDiagram(sites);
            Debug.Log("do we make it here?");
            GameObject diagramDisplay = new GameObject("VoronoiDiagramDisplay");
            SpriteRenderer spriteRenderer = diagramDisplay.AddComponent<SpriteRenderer>();
            spriteRenderer.sprite = Sprite.Create(diagramTexture, new Rect(0, 0, dim.x, dim.y), new Vector2(0.5f, 0.5f));

            diagramDisplay.transform.position = new Vector3(dim.x/2, heightAboveGround, dim.y/2);

        }

        */
        public Texture2D CreateVoronoiDiagram(List<Vector3> sites)
        {
            if (sites == null)
            {
                Debug.LogError("Sites list is null.");
                return null; // Or handle it accordingly
            }
            UnityEngine.Color[] regions = new UnityEngine.Color[sites.Count];
            for (int i = 0; i < sites.Count; i++)
            {
                regions[i] = new UnityEngine.Color(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), 1f);
            }

            UnityEngine.Color[] colors = new UnityEngine.Color[dim.x * dim.y];

            for (int y = 0; y < dim.y; y++)
            {

                for (int x = 0; x < dim.x; x++)
                {
                    int index = y * dim.x + x; //maybe x * dim.x + y
                    Vector3 pos = new Vector3(x, heightAboveGround, y);

                    colors[index] = regions[GetClosestSiteIndex(pos, sites)];
                    //Debug.Log("color index " + colors[index]);
                }

            }
            //Debug.Log("We are returning");
            return GetImageFromColorArray(colors);

        }

        int GetClosestSiteIndex(Vector3 pos, List<Vector3> sites)
        {
            float smallestDistance = float.MaxValue;
            int index = -1; // -1?
            for (int i = 0; i < sites.Count; i++)
            {
                float distance = Vector3.Distance(pos, sites[i]);
                if (distance < smallestDistance)
                {
                    smallestDistance = distance;
                    index = i;
                }
            }
            return index;
        }
        Texture2D GetImageFromColorArray(UnityEngine.Color[] colors)
        {

            Texture2D tex = new Texture2D(dim.x, dim.y);

            tex.filterMode = FilterMode.Point;

            tex.SetPixels(colors);

            tex.Apply();

            return tex;
        }

        /*
        Texture2D TestObstacles(List<Vector3> sites)
        {
            UnityEngine.Color[] regions = new UnityEngine.Color[sites.Count];
            UnityEngine.Color[] colors = new UnityEngine.Color[dim.x * dim.y];
            for (int i = 0; i < sites.Count; i++)
            {
                regions[i] = new UnityEngine.Color(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), 1f);
            }



        }
    */
        public static List<Vector3> ScaleSites(List<Vector3> originalSites, int resolutionMultiplier)
        {
            List<Vector3> scaledSites = new List<Vector3>();

            foreach (Vector3 site in originalSites)
            {
                // Assuming the original sites are in the range [0, 20] for both x and z.
                Vector3 scaledSite = new Vector3(site.x * resolutionMultiplier, site.y, site.z * resolutionMultiplier);
                scaledSites.Add(scaledSite);
            }

            return scaledSites;
        }

    }

}