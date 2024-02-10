using System.Collections.Generic;
using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car.Map;
using QuadTreePoint;

namespace PathPlanning
{
    class RRT
    {
        [Serializable]
        public class CostHistory
        {
            public List<float> costs = new List<float>();
            public List<int> nodeCounts = new List<int>();
            public List<float> timestamps = new List<float>();
        }
        class Node : IQuadTreeData
        {
            public Vector2 pos;
            public Vector2 vel;
            public List<Node> children;
            public Node parent;
            public List<PathSegment> pathFromParent;
            public bool atGoal = false;

            public float X { get { return pos.x; } }
            public float Y { get { return pos.y; } }

            public float Cost { get { return pathFromParent == null ? 0 : pathFromParent.Last().minCost; } }
            public PathSegment LastPath { get { return pathFromParent == null ? null : pathFromParent.Last(); } }
            public PathSegment FirstPath { get { return pathFromParent == null ? null : pathFromParent.First(); } }

            public Node(Vector2 pos, Vector2 vel, Node parent = null)
            {
                this.pos = pos;
                this.vel = vel;
                this.parent = parent;
                children = new List<Node>();
            }

            public void addChild(Node node)
            {
                children.Add(node);
            }
            public void removeChild(Node node)
            {
                children.Remove(node);
            }
        }

        CollisionDetector detector;
        List<Node> nodes;
        QuadTreeFloatPoint<Node> nodeTree;
        Rect mapBounds;
        List<Vector2> positions;
        List<float> times;
        Node endNode;
        List<Vector2> biasingPath;

        public LocalPlanner localPlanner;

        public float goalDistance = 7.5f; // Distance from goal to be considered reached
        public int kNearest = 20; // Number of near nodes to search for KNN
        public float searchRadius = 20f; // Search radius for area search
        
        public float biasRadius = 20f; // Furthest away from the biasing path to sample
        public bool goalPathBias = false; // If enabled, this concentrates sampling to within a radius of the minimum cost goal path
        public float biasProbability = 1.0f; // Samples along biasing path with a certain probability

        public bool areaSearch = true; // Search area around a new point instead of using KNN
        public bool pruning = false; // Enable tree pruning
        public bool startWithKnn = false; // Start by searching using KNN until a path is found, regardless of areaSearch

        public int pruneTarget = 10000; // Target number of nodes for pruning
        public int pruneThreshold = 1000; // Difference between number of nodes and prune target to trigger pruning
        public bool randomPruning = false; // Prunes random leaf nodes instead of the "worst" ones
       

        public CostHistory costHistory;
        bool foundGoal = false;
        System.Random random = new System.Random();

        public RRT()
        {
            this.localPlanner = new LocalPlanner();
        }
        public (List<Vector2>, List<float>) FindPath(MapManager mapManager, CollisionDetector detector, float timeLimit=20f)
        {
            this.detector = detector;
            this.nodes = new List<Node>();
            this.costHistory = new CostHistory();

            mapBounds = GetMapBounds();
            int bucketSize = 10;
            this.nodeTree = new QuadTreeFloatPoint<Node>(mapBounds.center.x, mapBounds.center.y, Mathf.Max(mapBounds.width, mapBounds.height)/2, bucketSize);

            List<Node> goalNodes = new List<Node>();

            Vector2 startPos = new Vector2(mapManager.GetGlobalStartPosition().x, mapManager.GetGlobalStartPosition().z);
            Vector2 goalPos = new Vector2(mapManager.GetGlobalGoalPosition().x, mapManager.GetGlobalGoalPosition().z);
            Vector2 startVel = new Vector2(0, 0.1f); // Just guessing that the car always points in the same direction

            Node root = new Node(startPos, startVel);
            nodes.Add(root);
            nodeTree.Insert(root);

            float goalMinCost = float.MaxValue;

            var stopwatch = System.Diagnostics.Stopwatch.StartNew();
            int nIterations = 0;
            while (true)
            {
                nIterations++;

                // Sample random point on map
                Vector2 p = SamplePoint();

                // Find closest nodes
                List<Node> nearNodes;
                if ((!foundGoal && startWithKnn) || !areaSearch)
                {
                    nearNodes = nodeTree.QueryNeighbours(p.x, p.y, kNearest);
                }
                else
                {
                    nearNodes = NodesWithinRadius(p, searchRadius);
                }

                // Determine the node with lowest cost
                float minCost = float.MaxValue;
                Node bestNode = null;
                List<PathSegment> bestPath = null;
                foreach (Node nearNode in nearNodes)
                {
                    if (nearNode.atGoal) continue; // Don't try to connect goal node

                    // Find path from nearNode to p
                    List<PathSegment> path = localPlanner.FindPathRelaxed(nearNode.pos, p, nearNode.vel, nearNode.LastPath);
                    if (path == null || detector.PathCollision(path))
                        continue;

                    // In rare cases we end up with an already existing end position, so we skip in that case
                    if (nodeTree.Exists(new Node(path.Last().p2, new Vector2(0,0))))
                    {
                        continue;
                    }

                    float cost = path.Last().minCost;
                    if (cost < minCost)
                    {
                        bestNode = nearNode;
                        bestPath = path;
                        minCost = cost;
                    }
                }
                if (bestNode == null)
                {
                    continue;
                }

                Node newNode = new Node(bestPath.Last().p2, bestPath.Last().endVel, bestNode);
                newNode.pathFromParent = bestPath;
                bestNode.addChild(newNode); // Add to RRT tree
                if ((!foundGoal && startWithKnn) || !areaSearch) nodeTree.Insert(newNode); // Add to quad tree (for proximity check)
                nodes.Add(newNode); // Add to list
                
                // Rewire tree
                foreach (Node nearNode in nearNodes)
                {
                    // Don't attempt to rewire to the bestNode or root node
                    if (nearNode == bestNode || nearNode.parent == null) continue;

                    // Find path from newNode to nearNode, needs to match end velocities so use Dubins path
                    List<PathSegment> path = localPlanner.FindDubinsPath(newNode.pos, nearNode.pos, newNode.vel, nearNode.vel, newNode.LastPath);
                    if (path == null || detector.PathCollision(path))
                        continue;

                    // Rewire if this path is better
                    float previousCost = nearNode.LastPath.minCost;
                    float previousLength = nearNode.LastPath.totalLength;
                    float newCost = path.Last().minCost;
                    float newLength = path.Last().totalLength;
                    if (newCost < previousCost)
                    {
                        // Make sure everything is accounted for when changing parent
                        newNode.addChild(nearNode);
                        nearNode.parent.removeChild(nearNode);
                        nearNode.parent = newNode;
                        nearNode.pathFromParent = path;
                        foreach (Node child in nearNode.children)
                        {
                            child.FirstPath.parent = path.Last();
                        }

                        // Propagate costs to children
                        float delta = newCost - previousCost;
                        float lengthDelta = newLength - previousLength;
                        UpdateCost(nearNode, delta, lengthDelta);
                    }
                }

                // Check if the node is at the goal
                if ((goalPos - bestPath.Last().p2).magnitude < goalDistance)
                {
                    // Found goal
                    newNode.atGoal = true;
                    foundGoal = true;
                    goalNodes.Add(newNode);
                }

                // Update best path and add to cost history
                if (goalNodes.Count > 0)
                {
                    Node bestGoalNode = LeastCostNode(goalNodes);
                    float cost = bestGoalNode.LastPath.minCost;
                    if (cost < goalMinCost)
                    {
                        // New best path
                        biasingPath = GetPointsInPath(bestGoalNode, 1f);

                        costHistory.costs.Add(cost);
                        costHistory.nodeCounts.Add(nodes.Count);
                        costHistory.timestamps.Add((float)stopwatch.Elapsed.TotalSeconds);

                        goalMinCost = cost;
                    }
                }

                // Prune the tree
                if (pruning && areaSearch && foundGoal && nodes.Count > (pruneTarget + pruneThreshold))
                {
                    while (nodes.Count > Math.Max(pruneTarget, goalNodes.Count))
                    {
                        float maxNormalizedCost = float.MinValue;
                        Node worstNode = null;
                        if (!randomPruning)
                        {
                            foreach (Node node in nodes)
                            {
                                if (node.children.Count == 0 && !node.atGoal)
                                {
                                    // Nodes with low cost compared to path length are considered better
                                    float normalizedCost = node.LastPath.minCost / node.LastPath.totalLength;
                                    if (normalizedCost > maxNormalizedCost)
                                    {
                                        worstNode = node;
                                        maxNormalizedCost = normalizedCost;
                                    }
                                }
                            }
                        }
                        else
                        {
                            do
                            {
                                worstNode = nodes[random.Next(0, nodes.Count)];
                            }
                            while (worstNode.children.Count > 0 || worstNode.atGoal);
                        }
                        
                        if (worstNode != null)
                        {
                            nodes.Remove(worstNode);
                            worstNode.parent.removeChild(worstNode);
                        }
                    }
                }

                // Stop after time limit
                if (stopwatch.Elapsed.TotalSeconds > timeLimit)
                {
                    Debug.Log("Timed out!");
                    break;
                }
            }
            stopwatch.Stop();

            endNode = LeastCostNode(goalNodes);

            Debug.Log($"Total number of nodes: {nodes.Count}");
            Debug.Log($"Final cost: {endNode.pathFromParent.Last().minCost} s");

            // Calculate number of path segments
            int nPaths = 0;
            PathSegment currentPath = endNode.LastPath;
            while (currentPath.parent != null)
            {
                nPaths++;
                currentPath = currentPath.parent;
            }
            Debug.Log($"Number of path segments: {nPaths}");

            (positions, times) = endNode.pathFromParent.Last().Render(localPlanner.maxSpeed, localPlanner.acceleration, localPlanner.deceleration, 0.1f);

            Debug.Log($"Rendered path time: {times.Last()} s");

            return (positions, times);
        }
        List<Node> NodesWithinRadius(Vector2 p, float radius)
        {
            List<Node> nearby = new List<Node>();
            foreach (Node node in nodes)
            {
                if ((node.pos - p).sqrMagnitude < radius * radius)
                {
                    nearby.Add(node);
                }
            }
            return nearby;
        }
        bool PointOutOfBounds(Vector2 p)
        {
            return p.x < mapBounds.xMin || p.x > mapBounds.xMax || p.y < mapBounds.yMin || p.y > mapBounds.yMax;
        }
        Vector2 SamplePoint()
        {
            if (!foundGoal || !goalPathBias || biasingPath == null)
            {
                return detector.SampleFree();
            }
            else
            {
                if (random.NextDouble() < biasProbability)
                {
                    // Sample within radius of best path to goal
                    Vector2 p;
                    do
                    {
                        int index = random.Next(0, biasingPath.Count);
                        Vector2 center = biasingPath[index];
                        float xOffset = ((float)random.NextDouble() * 2 - 1) * biasRadius;
                        float yOffset = ((float)random.NextDouble() * 2 - 1) * biasRadius;
                        p = center + new Vector2(xOffset, yOffset);
                    }
                    while (PointOutOfBounds(p) || detector.PointGridCollision(p));
                    return p;
                }
                else
                {
                    return detector.SampleFree();
                }
            }
        }
        List<Vector2> GetPointsInPath(Node endNode, float detail)
        {
            var list = new List<Vector2>();
            Node currentNode = endNode;

            while (currentNode.parent != null)
            {
                foreach (PathSegment path in currentNode.pathFromParent)
                {
                    int subdivisions = (int)Mathf.Ceil(path.length / detail);
                    for (int i = 0; i < subdivisions; i++)
                    {
                        list.Add(path.GetPosition(path.length*i / subdivisions));
                    }
                }
                currentNode = currentNode.parent;

            }
            return list;
        }
        Node LeastCostNode(List<Node> nodes)
        {
            float minCost = float.MaxValue;
            Node bestNode = null;
            foreach (Node node in nodes)
            {
                float cost = node.LastPath.minCost;
                if (cost < minCost)
                {
                    bestNode = node;
                    minCost = cost;
                }
            }
            return bestNode;
        }
        void UpdateCost(Node node, float prevDelta, float lengthDelta)
        {
            const float eps = 1e-6f;
            bool approx = false;
            float delta = 0;
            foreach (PathSegment path in node.pathFromParent)
            {
                if (approx)
                {
                    path.minCost += delta; // Delta is negative (hopefully)
                    path.totalLength += lengthDelta;
                }
                float originalCost = path.minCost;
                path.UpdateMinCost();
                delta = path.minCost - originalCost;
                
                if (Mathf.Abs(delta - prevDelta) < eps)
                {
                    approx = true;
                }
            }
            if (approx) {
                foreach (Node child in node.children)
                {
                    UpdateCostApprox(child, delta, lengthDelta);
                }
            }
            else
            {
                foreach (Node child in node.children)
                {
                    UpdateCost(child, delta, lengthDelta);
                }
            }
        }
        void UpdateCostApprox(Node node, float delta, float lengthDelta)
        {
            foreach (PathSegment path in node.pathFromParent)
            {
                path.minCost += delta;
                path.totalLength += lengthDelta;
            }
            foreach (Node child in node.children)
            {
                UpdateCostApprox(child, delta, lengthDelta);
            }
        }
        Rect GetMapBounds()
        {
            
            float mapMinX = float.MaxValue;
            float mapMaxX = float.MinValue;
            float mapMinY = float.MaxValue;
            float mapMaxY = float.MinValue;
            foreach (Rect rect in detector.boundingBoxes)
            {
                if (rect.xMin < mapMinX) mapMinX = rect.xMin;
                if (rect.xMax > mapMaxX) mapMaxX = rect.xMax;
                if (rect.yMin < mapMinY) mapMinY = rect.yMin;
                if (rect.yMax > mapMaxY) mapMaxY = rect.yMax;
            }

            Rect mapBounds = new Rect();
            mapBounds.xMin = mapMinX;
            mapBounds.xMax = mapMaxX;
            mapBounds.yMin = mapMinY;
            mapBounds.yMax = mapMaxY;

            return mapBounds;
        }
        public void DebugDrawTree()
        {
            foreach (Node node in nodes)
            {
                if (node.pathFromParent == null) continue;
                foreach (PathSegment path in node.pathFromParent)
                {
                    path.DebugDraw(Color.red);
                }
            }
        }
        public void DebugDrawPath()
        {
            Node currentNode = endNode;
            while (currentNode.parent != null)
            {
                Color color = Color.magenta;
                if (currentNode.pathFromParent.Count == 3)
                {
                    color = Color.green;
                }
                foreach (PathSegment path in currentNode.pathFromParent)
                {
                    path.DebugDraw(color);
                }

                currentNode = currentNode.parent;
            }
            for (int i = 0; i < positions.Count; i++)
            {
                if (i > 0)
                {
                    //DebugDrawLine(positions[i-1], positions[i], Color.magenta);
                    float velocity = (positions[i] - positions[i - 1]).magnitude / (times[i] - times[i - 1]);
                    Vector2 pos = positions[i - 1];
                    Debug.DrawLine(new Vector3(pos.x, 0.3f, pos.y), new Vector3(pos.x, velocity, pos.y), Color.cyan, 1000f);
                }
            }
        }
        void DebugDrawLine(Vector2 a, Vector2 b, Color color)
        {
            Debug.DrawLine(new Vector3(a.x, 0.3f, a.y), new Vector3(b.x, 0.3f, b.y), color, 1000f);
        }
    }
    
}
