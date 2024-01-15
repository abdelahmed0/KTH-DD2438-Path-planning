using System.IO;
using Newtonsoft.Json;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car.Map
{
    public static class FileUtils
    {
        public static UnityEngine.Object LoadPrefabFromFile(string filename)
        {
            var loadedObject = Resources.Load("Prefabs/" + filename);
            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/Platformer/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitCommercial/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitSuburban/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/CityKitRoads/" + filename);
            }

            if (loadedObject == null)
            {
                loadedObject = Resources.Load("Prefabs/Kenneys/Nature/" + filename);
            }

            if (loadedObject == null)
            {
                throw new FileNotFoundException("No file found trying to load Prefaob from file (" + filename + ")... - please check the configuration");
            }

            return loadedObject;
        }

        public static TerrainInfo CreateTerrainInfoFromJSONFileLegacy(string terrain_filename)
        {
            var jsonTextFile = Resources.Load<TextAsset>("Text/" + terrain_filename);
            return JsonConvert.DeserializeObject<TerrainInfo>(jsonTextFile.text);
        }

        public static string ReadJsonFromFile(string filename)
        {
            string textContent = "";

            if (filename != null && filename.Length > 0)
            {
                string path = Application.dataPath + "/Resources/Text/" + filename + ".json";
                textContent = File.ReadAllText(path);
            }

            if (textContent.Length == 0)
            {
                throw new FileNotFoundException("No file found trying to load text from file (" + filename + ")... - please check the configuration");
            }

            return textContent;
        }

        public static void WriteJsonToFile(string jsonString, string filename)
        {
            if (filename != null && filename.Length > 0)
            {
                string path = Application.dataPath + "/Resources/Text/" + filename + ".json";
                Debug.Log("Writing to AssetPath:" + path);
                File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
                UnityEditor.AssetDatabase.Refresh();
#endif
            }
        }
    }
}