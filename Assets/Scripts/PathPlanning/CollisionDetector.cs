using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Diagnostics;
using util;

namespace PathPlanning
{
    class CollisionDetector
    {
        public List<Rect> boundingBoxes;
        private float yMin;
        private float yMax;

        public Dictionary<Vector2Int, bool> traversabilityMap;
        public float cellSize;

        public List<Vector2Int> freeCells;
        System.Random rand = new System.Random();
        
        public CollisionDetector(MapManager mapManager, float margin = 1f, float cellSize = 1.25f, float yMin=-0.1f, float yMax=1.75f)
        {
            this.yMin = yMin;
            this.yMax = yMax;
            this.cellSize = cellSize;

            this.boundingBoxes = new List<Rect>();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            foreach (GameObject gameObject in mapManager.GetObstacleMap().obstacleObjects)
            {
                if (gameObject.name.Contains("road")) continue;

                if (gameObject.TryGetComponent<MeshCollider>(out var meshCollider))
                {
                    Mesh mesh = meshCollider.sharedMesh;
                    Mesh readableMesh = MeshUtil.MakeReadableMeshCopy(mesh);
                    boundingBoxes.AddRange(MeshUtil.BoundingBoxesFromMesh(readableMesh, gameObject.transform, margin, yMin, yMax));
                }
            }
            sw.Stop();
            UnityEngine.Debug.Log($"Generate bounding boxes: {sw.ElapsedMilliseconds} ms");
            UnityEngine.Debug.Log($"Number of bounding boxes: {boundingBoxes.Count}");

            // Fill traversability map
            sw.Restart();
            (traversabilityMap, freeCells) = CreateTraversabilityMap(cellSize);
            sw.Stop();
            UnityEngine.Debug.Log($"Create maps: {sw.ElapsedMilliseconds} ms");
        }

        public (Dictionary<Vector2Int, bool>, List<Vector2Int>) CreateTraversabilityMap(float cellSize)
        {
            var traversabilityMap = new Dictionary<Vector2Int, bool>();
            var freeCells = new List<Vector2Int>();

            float mapMinX = float.MaxValue;
            float mapMaxX = float.MinValue;
            float mapMinY = float.MaxValue;
            float mapMaxY = float.MinValue;
            foreach (Rect rect in boundingBoxes)
            {
                if (rect.xMin < mapMinX) mapMinX = rect.xMin;
                if (rect.xMax > mapMaxX) mapMaxX = rect.xMax;
                if (rect.yMin < mapMinY) mapMinY = rect.yMin;
                if (rect.yMax > mapMaxY) mapMaxY = rect.yMax;

                Vector2Int gridMin = GetGridPos(rect.min, cellSize);
                Vector2Int gridMax = GetGridPos(rect.max, cellSize);

                for (int x = gridMin.x; x <= gridMax.x; x++)
                {
                    for (int y = gridMin.y; y <= gridMax.y; y++)
                    {
                        traversabilityMap[new Vector2Int(x, y)] = true;
                    }
                }
            }

            // Fill empty space
            Vector2Int mapMin = GetGridPos(new Vector2(mapMinX, mapMinY), cellSize);
            Vector2Int mapMax = GetGridPos(new Vector2(mapMaxX, mapMaxY), cellSize);
            for (int x = mapMin.x; x <= mapMax.x; x++)
            {
                for (int y = mapMin.y; y <= mapMax.y; y++)
                {
                    Vector2Int mapPos = new Vector2Int(x, y);
                    if (!traversabilityMap.ContainsKey(mapPos))
                    {
                        traversabilityMap[mapPos] = false;
                        freeCells.Add(mapPos);
                    }
                }
            }
            return (traversabilityMap, freeCells);
        }

        public Vector2 SampleFree()
        {
            int index = rand.Next(0, freeCells.Count);
            Vector2Int cell = freeCells[index];
            Vector2 pos = GetWorldPos(cell, cellSize);

            float randX = UnityEngine.Random.Range(-cellSize, cellSize) / 2;
            float randY = UnityEngine.Random.Range(-cellSize, cellSize) / 2;
            return new Vector2(pos.x + randX, pos.y + randY);
        }
        public Vector2Int GetGridPos(Vector2 pos, float cellSize)
        {
            return Vector2Int.FloorToInt(pos / cellSize);
        }
        public Vector2 GetWorldPos(Vector2Int gridPos, float cellSize)
        {
            return new Vector2((gridPos.x + 0.5f) * cellSize, (gridPos.y + 0.5f) * cellSize);
        }
        public bool PathCollision(List<PathSegment> paths)
        {
            foreach (PathSegment path in paths) 
            {
                if (PathCollision(path)) return true;            
            }
            return false;
        }
        public bool PathCollision(PathSegment anyPath)
        {
            if (anyPath is CirclePathSegment path)
            {
                return CircleSegmentCollision(path.p1, path.p2, path.c, path.radius, path.angle, path.sgn);
            }
            else
            {
                return LineCollision(anyPath.p1, anyPath.p2);
            }
        }
        
        // Fast point collision
        public bool PointGridCollision(Vector2 p)
        {
            return traversabilityMap[GetGridPos(p, cellSize)];
        }
        public bool PointCollision(Vector2 p)
        {
            foreach (Rect rect in boundingBoxes)
            {
                if (PointRectCollision(p, rect))
                {
                    return true;
                }
            }
            return false;
        }
        public bool LineCollision(Vector2 p1, Vector2 p2)
        {
            foreach (Rect rect in boundingBoxes)
            {
                if (LineRectCollision(p1, p2, rect))
                {
                    return true;
                }
            }
            return false;
        }
        public bool CircleSegmentCollision(Vector2 p1, Vector2 p2, Vector2 c, float r, float angle, int sgn)
        {
            foreach (Rect rect in boundingBoxes)
            {
                if (CircleSegmentRectCollision(p1, p2, c, r, angle, sgn, rect))
                {
                    return true;
                }
            }
            return false;
        }
        bool PointRectCollision(Vector2 p, Rect rect)
        {
            return p.x < rect.xMax && p.x > rect.xMin && p.y < rect.yMax && p.y > rect.yMin;
        }

        // Taken from https://stackoverflow.com/questions/99353/how-to-test-if-a-line-segment-intersects-an-axis-aligned-rectange-in-2d
        bool LineRectCollision(Vector2 a, Vector2 b, Rect rect)
        {
            if (a.x > rect.xMax && b.x > rect.xMax) return false;
            if (a.x < rect.xMin && b.x < rect.xMin) return false;
            if (a.y > rect.yMax && b.y > rect.yMax) return false;
            if (a.y < rect.yMin && b.y < rect.yMin) return false;

            float A = b.y - a.y;
            float B = a.x - b.x;
            float C = b.x*a.y - a.x*b.y;

            bool tr = A*rect.xMax + B*rect.yMax + C > 0;
            bool tl = A*rect.xMin + B*rect.yMax + C > 0;
            bool br = A*rect.xMax + B*rect.yMin + C > 0;
            bool bl = A*rect.xMin + B*rect.yMin + C > 0;

            return (tr != tl || tl != br || br != bl || tr != br || tl != bl || tr != bl);
            //return (tr || tl || br || bl) && (!tr || !tl || !br || !bl);
        }
        bool CheckCircleIntersect(float a, float b, float c, Vector2 lineStart, Vector2 d, Vector2 start, int sgn, float angle)
        {
            float D = b*b - 4*a*c;
            if (D < 0) return false;

            float rootD = Mathf.Sqrt(D);
            float lambda1 = 0.5f * (-b + rootD) / a;
            float lambda2 = 0.5f * (-b - rootD) / a;
            if (lambda1 > 0 && lambda1 < 1)
            {
                Vector2 pc = lineStart + lambda1*d;
                if (AngleOnCircle(start, pc, sgn) < angle) return true;
            }
            if (lambda2 > 0 && lambda2 < 1)
            {
                Vector2 pc = lineStart + lambda2 * d;
                if (AngleOnCircle(start, pc, sgn) < angle) return true;
            }
            return false;
        }
        // Partly from https://math.stackexchange.com/questions/2536048/circle-line-intersection
        bool CircleSegmentRectCollision(Vector2 p1, Vector2 p2, Vector2 c, float r, float angle, int sgn, Rect rect)
        {
            // Check if circle is completely outside the bounding box
            float xMax = rect.xMax + r;
            float xMin = rect.xMin - r;
            float yMax = rect.yMax + r;
            float yMin = rect.yMin - r;

            if (c.x > xMax || c.x < xMin || c.y > yMax || c.y < yMin) return false;
            
            float x1 = rect.xMin - c.x;
            float x2 = rect.xMax - c.x;
            float y1 = rect.yMin - c.y;
            float y2 = rect.yMax - c.y;

            Vector2[] ps = { new Vector2(x1, y1), new Vector2(x1, y2), new Vector2(x1, y1), new Vector2(x2, y1) };
            Vector2[] qs = { new Vector2(x2, y1), new Vector2(x2, y2), new Vector2(x1, y2), new Vector2(x2, y2) };

            for (int i = 0; i < 4; i++)
            {
                Vector2 p = ps[i];
                Vector2 q = qs[i];
                Vector2 d = q - p;

                float a = d.x * d.x + d.y * d.y;
                float b = 2 * Vector2.Dot(p, d);
                float C = Vector2.Dot(p, p) - r*r;

                if (CheckCircleIntersect(a, b, C, p, d, p1 - c, sgn, angle)) return true;
            }

            /*float dx = x2 - x1;
            float ax = dx*dx;
            float bx = 2*x1*dx;
            
            float ctl = x1*x1 + y2*y2 - r*r;
            float cbl = x1 * x1 + y1 * y1 - r * r;

            Vector2 bl = new Vector2(x1, y1);
            Vector2 tl = new Vector2(x1, y2);
            Vector2 dxVec = new Vector2(dx, 0);

            if (CheckCircleIntersect(ax, bx, ctl, tl, dxVec, p1-c, sgn, angle)) return true;
            if (CheckCircleIntersect(ax, bx, cbl, bl, dxVec, p1-c, sgn, angle)) return true;

            float dy = y2 - y1;
            float ay = dy * dy;
            float by = 2*y1 * dy;

            float cbr = x2 * x2 + y1 * y1 - r * r;

            Vector2 br = new Vector2(x2, y1);
            Vector2 dyVec = new Vector2(0, dy);

            if (CheckCircleIntersect(ay, by, cbl, bl, dyVec, p1, sgn, angle)) return true;
            if (CheckCircleIntersect(ay, by, cbr, br, dyVec, p1, sgn, angle)) return true;*/

            return false;
        }

        float AngleOnCircle(Vector2 p1, Vector2 p2, int sgn)
        {
            // Calculates angle traversed between two points on circle given the driving direction
            float theta = Mathf.Atan2(p2.y, p2.x) - Mathf.Atan2(p1.y, p1.x);
            if (theta < 0 && sgn == -1)
                theta += 2 * Mathf.PI;
            else if (theta > 0 && sgn == 1)
                theta -= 2 * Mathf.PI;

            return Mathf.Abs(theta);
        }
        public void DebugDrawBoundingBoxes()
        {
            foreach(Rect rect in boundingBoxes)
            {
                Gizmos.color = Color.magenta;

                Gizmos.DrawCube(new Vector3(rect.center.x, (yMax+yMin)/2, rect.center.y), new Vector3(rect.width, yMax-yMin, rect.height));
                
            }
        }
        public void DebugDrawGrid()
        {
            foreach (Vector2Int key in traversabilityMap.Keys)
            {
                Gizmos.color = traversabilityMap[key] ? Color.red : Color.green;
                Vector3 pos = new Vector3((key.x + 0.5f) * cellSize, 20f, (key.y + 0.5f) * cellSize);
                Gizmos.DrawCube(pos, new Vector3(cellSize*0.9f, 0.1f, cellSize*0.9f));
            }
        }
    }
}
