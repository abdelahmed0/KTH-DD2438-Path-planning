using System;
using UnityEditor;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car.Map
{
    [CustomEditor(typeof(MapManager))]
    public class ObjectBuilderEditor : Editor
    {
        private void OnEnable()
        {
            MapManager myScript = (MapManager)target;
            myScript.Initialize();
        }

        public override void OnInspectorGUI()
        {
            var changed = DrawDefaultInspector();

            MapManager myScript = (MapManager)target;
            if (changed)
            {
                myScript.Initialize();
            }


            if (GUILayout.Button("Save") && myScript.fileName.Length > 0)
            {
                myScript.SaveMap();
            }

            if (GUILayout.Button("Load") && myScript.fileName.Length > 0)
            {
                myScript.LoadMap();
            }

            if (GUILayout.Button("Load Legacy") && myScript.fileName.Length > 0)
            {
                myScript.LoadLegacyMap();
            }
            
            if (GUILayout.Button("Clear"))
            {
                myScript.ClearMap();
            }   
        }
    }
}