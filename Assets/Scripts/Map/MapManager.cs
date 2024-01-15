using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Tilemaps;

namespace UnityStandardAssets.Vehicles.Car.Map
{
    public class MapManager : MonoBehaviour
    {
        public Grid grid;
        public GameObject groundPlane;

        public Vector3 localStartPosition;
        public Vector3 localGoalPosition;

        public String fileName;

        private ObstacleMap ObstacleMap;
        public bool DrawObstacleMap;

        private List<Tilemap> tilemaps;

        private void Start()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (grid != null)
            {
                tilemaps = grid.GetComponentsInChildren<Tilemap>().ToList().FindAll(tilemap => tilemap.enabled);
                var gameObjects = tilemaps
                    .Select(tilemap => tilemap.transform)
                    .SelectMany(transform =>
                        {
                            var ret = new List<GameObject>();
                            foreach (Transform child in transform)
                            {
                                ret.Add(child.gameObject);
                            }

                            return ret;
                        }
                    ).Where(obj => obj.activeInHierarchy)
                    .ToList();

                ObstacleMap = new ObstacleMap(
                    gameObjects,
                    grid);
            }
        }

        public ObstacleMap GetObstacleMap()
        {
            if (ObstacleMap == null) Initialize();
            return ObstacleMap;
        }

        public Vector3 GetGlobalStartPosition()
        {
            return grid.transform.TransformPoint(localStartPosition);
        }

        public Vector3 GetGlobalGoalPosition()
        {
            return grid.transform.TransformPoint(localGoalPosition);
        }

        internal void ClearMap()
        {
            foreach (var tilemap in tilemaps)
            {
                List<GameObject> children = FetchChildren(tilemap.gameObject);
                foreach (var child in children)
                {
                    DestroyImmediate(child);
                }

                DestroyImmediate(tilemap.gameObject);
            }

            groundPlane.transform.localScale = Vector3.zero;
            tilemaps = new List<Tilemap>();
        }

        public void SaveMap()
        {
            if (tilemaps.Count == 0) return;

            var saveInfo = PrepareSaveData();

            var json = EditorJsonUtility.ToJson(saveInfo);
            FileUtils.WriteJsonToFile(json, fileName);
        }

        public void LoadMap()
        {
            var saveData = new SaveData();
            EditorJsonUtility.FromJsonOverwrite(FileUtils.ReadJsonFromFile(fileName), saveData);
            LoadSaveData(saveData);
        }

        public void LoadLegacyMap()
        {
            var saveData = new SaveData();
            var terrainInfo = FileUtils.CreateTerrainInfoFromJSONFileLegacy(fileName);

            ClearMap();

            float x_step = (terrainInfo.x_high - terrainInfo.x_low) / terrainInfo.x_N;
            float z_step = (terrainInfo.z_high - terrainInfo.z_low) / terrainInfo.z_N;
            saveData.grid = new SaveData.SavedGrid();
            saveData.grid.scale = new Vector3(x_step, 15.0f, z_step);
            saveData.goalPosition = new Vector3(terrainInfo.goal_pos.x / saveData.grid.scale.x,
                terrainInfo.goal_pos.y / saveData.grid.scale.y, terrainInfo.goal_pos.z / saveData.grid.scale.z);
            saveData.startPosition = new Vector3(terrainInfo.start_pos.x / saveData.grid.scale.x,
                terrainInfo.start_pos.y / saveData.grid.scale.y + 0.05f,
                terrainInfo.start_pos.z / saveData.grid.scale.z);

            saveData.savedTileMaps = new List<SaveData.SavedTilemap>();
            var savedTilemap = new SaveData.SavedTilemap();
            savedTilemap.name = "Ground 0";
            savedTilemap.position = Vector3.zero;
            savedTilemap.scale = Vector3.one;


            var savedTiles = new List<SaveData.SavedTile>();
            for (int i = 0; i < terrainInfo.x_N; i++)
            {
                for (int j = 0; j < terrainInfo.z_N; j++)
                {
                    if (terrainInfo.traversability[i, j] > 0.5f)
                    {
                        var savedTile = new SaveData.SavedTile();

                        savedTile.name = "block";
                        savedTile.hasCollider = true;
                        savedTile.isConvex = true;
                        savedTile.position = new Vector3(terrainInfo.get_x_pos(i) / saveData.grid.scale.x, 0.0f,
                            terrainInfo.get_z_pos(j) / saveData.grid.scale.z);
                        savedTile.rotation = Quaternion.identity;
                        savedTile.scale = Vector3.one;

                        savedTiles.Add(savedTile);
                    }
                }
            }

            //TODO: Calc based on actual placement of tiles
            saveData.groundPosition = new Vector3((terrainInfo.x_low + terrainInfo.x_high) / 2, 0.0f,
                (terrainInfo.z_high + terrainInfo.z_low) / 2);
            saveData.groundScale = new Vector3((terrainInfo.x_high - terrainInfo.x_low), 1,
                (terrainInfo.z_high - terrainInfo.z_low));

            savedTilemap.savedTiles = savedTiles;
            saveData.savedTileMaps.Add(savedTilemap);

            LoadSaveData(saveData);
        }

        private SaveData PrepareSaveData()
        {
            var saveData = new SaveData();
            saveData.savedTileMaps = new List<SaveData.SavedTilemap>();
            saveData.groundScale = groundPlane.transform.localScale;
            saveData.groundPosition = groundPlane.transform.localPosition;

            saveData.startPosition = localStartPosition;
            saveData.goalPosition = localGoalPosition;

            saveData.grid = new SaveData.SavedGrid
            {
                scale = grid.transform.localScale
            };

            foreach (var tilemap in tilemaps)
            {
                var saveMap = new SaveData.SavedTilemap();
                saveMap.name = tilemap.name;
                saveMap.scale = tilemap.transform.localScale;
                saveMap.position = tilemap.transform.localPosition;

                saveMap.savedTiles = new List<SaveData.SavedTile>();

                foreach (Transform child in tilemap.transform)
                {
                    var childGameObject = child.gameObject;
                    var meshCollider = childGameObject.GetComponent<MeshCollider>();
                    saveMap.savedTiles.Add(new SaveData.SavedTile
                    {
                        name = childGameObject.name,
                        position = childGameObject.transform.localPosition,
                        rotation = childGameObject.transform.localRotation,
                        scale = childGameObject.transform.localScale,
                        hasCollider = meshCollider != null,
                        isConvex = meshCollider != null && meshCollider.convex,
                    });
                }

                saveData.savedTileMaps.Add(saveMap);
            }

            return saveData;
        }

        private void LoadSaveData(SaveData saveData)
        {
            ClearMap();

            grid.transform.localScale = saveData.grid.scale;
            grid.cellSwizzle = GridLayout.CellSwizzle.XZY;

            groundPlane.transform.localScale = saveData.groundScale;
            groundPlane.transform.localPosition = saveData.groundPosition;

            localStartPosition = saveData.startPosition;
            localGoalPosition = saveData.goalPosition;

            foreach (var savedTileMap in saveData.savedTileMaps)
            {
                var tilemapObj = new GameObject("Cool GameObject made from Code");
                tilemapObj.transform.parent = grid.transform;

                tilemapObj.name = savedTileMap.name;
                tilemapObj.transform.localScale = savedTileMap.scale;
                tilemapObj.transform.localPosition = savedTileMap.position;

                var tilemap = tilemapObj.AddComponent<Tilemap>();
                tilemapObj.AddComponent<TilemapRenderer>();
                tilemap.orientation = Tilemap.Orientation.XZ;
                tilemaps.Add(tilemap);

                foreach (var savedTile in savedTileMap.savedTiles)
                {
                    // Called via:
                    var loadedPrefabResource = FileUtils.LoadPrefabFromFile(savedTile.name);
                    var instantiated = (GameObject)Instantiate(loadedPrefabResource, Vector3.zero, Quaternion.identity);
                    instantiated.transform.parent = tilemapObj.transform;

                    instantiated.name = savedTile.name;
                    instantiated.transform.localRotation = savedTile.rotation;
                    instantiated.transform.localScale = savedTile.scale;
                    instantiated.transform.localPosition = savedTile.position;
                    if (savedTile.hasCollider)
                    {
                        var collider = instantiated.AddComponent<MeshCollider>();
                        collider.convex = savedTile.isConvex;
                    }
                }
            }
        }


        private List<GameObject> FetchChildren(GameObject gameObject)
        {
            List<GameObject> children = new List<GameObject>();
            foreach (Transform child in gameObject.transform)
            {
                children.Add(child.gameObject);
            }

            return children;
        }

        void OnDrawGizmos()
        {
            if (DrawObstacleMap)
            {
                if (ObstacleMap == null)
                {
                    Initialize();
                }

                foreach (var posEntity in ObstacleMap.traversabilityPerCell)
                {
                    var position = new Vector3Int(posEntity.Key.x, 1, posEntity.Key.y);

                    var cellToLocal = grid.CellToLocal(position);
                    cellToLocal = new Vector3(cellToLocal.x, 1, cellToLocal.y);
                    cellToLocal += grid.cellSize / 2;
                    cellToLocal = grid.transform.TransformPoint(cellToLocal);

                    var gizmoSize = grid.cellSize;
                    gizmoSize.y = 0.005f;
                    gizmoSize.Scale(grid.transform.localScale*0.8f);
                    if (posEntity.Value == ObstacleMap.Traversability.Blocked)
                    {
                        Gizmos.color = Color.red;
                        Gizmos.DrawCube(cellToLocal, gizmoSize);
                    }
                    else if (posEntity.Value == ObstacleMap.Traversability.Partial)
                    {
                        Gizmos.color = Color.yellow;
                        Gizmos.DrawCube(cellToLocal, gizmoSize);
                    }
                    else
                    {
                        Gizmos.color = Color.green;
                        Gizmos.DrawCube(cellToLocal, gizmoSize);
                    }
                }
            }
            else
            {
                ObstacleMap = null;
            }
        }
    }


    [Serializable]
    public class SaveData
    {
        public SavedGrid grid;
        public List<SavedTilemap> savedTileMaps;

        public Vector3 groundScale;
        public Vector3 groundPosition;

        public Vector3 startPosition;
        public Vector3 goalPosition;

        [Serializable]
        public struct SavedTilemap
        {
            public String name;
            public Vector3 scale;
            public Vector3 position;
            public List<SavedTile> savedTiles;
        }

        [Serializable]
        public struct SavedTile
        {
            public String name;
            public Vector3 position;
            public Quaternion rotation;
            public Vector3 scale;

            public bool hasCollider;
            public bool isConvex;
        }

        [Serializable]
        public struct SavedGrid
        {
            public Vector3 scale;
        }
    }
}