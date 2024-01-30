using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions.Must;

namespace UnityStandardAssets.Vehicles.Car.Map
{
    public class ObstacleMap
    {
        private List<GameObject> obstacles;

        public readonly Dictionary<Vector2Int, Traversability> traversabilityPerCell;
        public readonly Dictionary<Vector2Int, List<GameObject>> gameGameObjectsPerCell;
        public readonly List<GameObject> obstacleObjects;
        public readonly Grid grid;
        public readonly BoundsInt mapBounds;

        public ObstacleMap(List<GameObject> obstacleObjects, Grid grid)
        {
            if (grid.cellSize.x == 0 || grid.cellSize.y == 0) throw new ArgumentException("Invalid Grid size. Cannot be 0!");

            this.grid = grid;
            this.obstacleObjects = obstacleObjects;

            var mapBoundsHelper = obstacleObjects[0].GetComponent<Renderer>().bounds;
            foreach (Renderer renderer in obstacleObjects.Select(obj => obj.GetComponent<Renderer>()))
            {
                mapBoundsHelper.Encapsulate(renderer.bounds);
            }

            mapBoundsHelper = InverseTransformBounds(grid.transform, mapBoundsHelper);
            mapBounds = new BoundsInt(
                Vector3Int.CeilToInt(new Vector3(mapBoundsHelper.min.x / grid.cellSize.x, 0, mapBoundsHelper.min.z / grid.cellSize.y)),
                Vector3Int.FloorToInt(new Vector3(mapBoundsHelper.size.x / grid.cellSize.x, 1, mapBoundsHelper.size.z / grid.cellSize.y))
            );

            (gameGameObjectsPerCell, traversabilityPerCell) = GenerateMapData(obstacleObjects);
        }

        public Bounds InverseTransformBounds(Transform _transform, Bounds _localBounds)
        {
            var center = _transform.InverseTransformPoint(_localBounds.center);

            var extents = _localBounds.extents;
            var axisX = _transform.InverseTransformVector(extents.x, 0, 0);
            var axisY = _transform.InverseTransformVector(0, extents.y, 0);
            var axisZ = _transform.InverseTransformVector(0, 0, extents.z);

            extents.x = Mathf.Abs(axisX.x) + Mathf.Abs(axisY.x) + Mathf.Abs(axisZ.x);
            extents.y = Mathf.Abs(axisX.y) + Mathf.Abs(axisY.y) + Mathf.Abs(axisZ.y);
            extents.z = Mathf.Abs(axisX.z) + Mathf.Abs(axisY.z) + Mathf.Abs(axisZ.z);

            return new Bounds { center = center, extents = extents };
        }
        public Traversability IsLocalPointTraversable(Vector3 localPosition)
        {
            var cellPos = grid.LocalToCell(localPosition);
            return traversabilityPerCell[new Vector2Int(cellPos.x, cellPos.y)];
        }

        private (Dictionary<Vector2Int, List<GameObject>>, Dictionary<Vector2Int, Traversability>) GenerateMapData(List<GameObject> gameObjects)
        {
            var gameObjectsPerCell = new Dictionary<Vector2Int, List<GameObject>>();
            var traversabilityData = new Dictionary<Vector2Int, Traversability>();

            foreach (var pos in mapBounds.allPositionsWithin)
            {
                gameObjectsPerCell[new Vector2Int(pos.x, pos.z)] = new List<GameObject>();
                traversabilityData[new Vector2Int(pos.x, pos.z)] = Traversability.Free;
            }


            foreach (var gameObject in gameObjects)
            {
                var bounds = InverseTransformBounds(grid.transform, gameObject.GetComponent<Renderer>().bounds);

                BoundsInt boundsInt;
                if (gameObject.name.Contains("block"))
                {
                    boundsInt = new BoundsInt(
                        Vector3Int.CeilToInt(new Vector3(bounds.min.x / grid.cellSize.x, 0, bounds.min.z / grid.cellSize.y)),
                        Vector3Int.FloorToInt(new Vector3(bounds.size.x / grid.cellSize.x, 1, bounds.size.z / grid.cellSize.y)));
                }
                else
                {
                    bounds.min = Vector3Int.FloorToInt(new Vector3(bounds.min.x / grid.cellSize.x, 0, bounds.min.z / grid.cellSize.y));
                    bounds.max = Vector3Int.CeilToInt(new Vector3(bounds.max.x / grid.cellSize.x, 1, bounds.max.z / grid.cellSize.y));
                    // Resize bounds first, then take min max.
                    
                    boundsInt = new BoundsInt(Vector3Int.FloorToInt(bounds.min), Vector3Int.CeilToInt(bounds.size));
                }


                foreach (var cellPosition in boundsInt.allPositionsWithin)
                {
                    var dictVector = new Vector2Int(cellPosition.x, cellPosition.z);
                    if (gameObject.name.Contains("road"))
                    {
                        continue;
                    }

                    if (boundsInt.min.x != dictVector.x &&
                        boundsInt.min.z != dictVector.y &&
                        boundsInt.max.x - 1 != dictVector.x &&
                        boundsInt.max.z - 1 != dictVector.y)
                    {
                        traversabilityData[dictVector] = Traversability.Blocked;
                        gameObjectsPerCell[dictVector].Add(gameObject);
                    }
                    else if (!traversabilityData.ContainsKey(dictVector) || traversabilityData[dictVector] != Traversability.Blocked)
                    {
                        traversabilityData[dictVector] = Traversability.Partial;
                        gameObjectsPerCell[dictVector].Add(gameObject);
                    }

                    if (gameObject.name.Contains("block"))
                    {
                        traversabilityData[dictVector] = Traversability.Blocked;
                    }
                }
            }

            return (gameObjectsPerCell, traversabilityData);
        }


        public enum Traversability
        {
            Free = 0,
            Partial = 1,
            Blocked = 2,
        }
    }
}