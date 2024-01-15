using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine.Serialization;
using UnityStandardAssets.Vehicles.Car.Map;

public class GameManager : MonoBehaviour
{
    public MapManager mapManager;
    public GameObject vehiclePrefab;

    float startTime;
    public float completionTime;
    public float goalTolerance = 10.0f;
    public bool finished = false;

    public Camera driveCamera;

    public GameObject replayVehiclePrefab;

    public bool MassReplay;


    public string massReplayKeyword = "car";

    private GameObject vehicleInstance;

    void Start()
    {
        startTime = Time.time;

        vehicleInstance = Instantiate(vehiclePrefab);

        var mapManagerStartPosition = mapManager.GetGlobalStartPosition();
        mapManagerStartPosition.Scale(mapManager.grid.transform.localScale);

        vehicleInstance.transform.position = mapManager.GetGlobalStartPosition();
        vehicleInstance.transform.rotation = Quaternion.identity;
        vehicleInstance.transform.parent = mapManager.grid.transform;

        var followObject = driveCamera.GetComponent<FollowObject>();
        if (followObject != null)
        {
            followObject.target_object = vehicleInstance.transform;
            followObject.CameraFixed = vehicleInstance.name.ToLower().Contains("drone");
            followObject.Start();
        }

        if (MassReplay)
            SetupReplay();
    }

    // Update is called once per frame
    void Update()
    {
        if (!finished)
        {
            if ((vehicleInstance.transform.position - mapManager.GetGlobalGoalPosition()).magnitude < goalTolerance)
            {
                completionTime = Time.time - startTime;
                finished = true;
            }
        }
    }

    void SetupReplay()
    {
        string folderName = Directory.GetCurrentDirectory() + "/Assets/Resources/Text";
        if (Directory.Exists(folderName))
        {
            GameObject spawn_vehicle;
            TrajectoryLogger spawn_logger;
            Text spawn_text;
            string group_number;

            DirectoryInfo d = new DirectoryInfo(folderName);
            foreach (var file in d.GetFiles("*.json"))
            {
                if (file.Name.ToLower().Contains(massReplayKeyword.ToLower()))
                {
                    Debug.Log(file.Name);
                    spawn_vehicle = Instantiate(replayVehiclePrefab, mapManager.GetGlobalStartPosition(),
                        Quaternion.identity);
                    spawn_logger = spawn_vehicle.GetComponent<TrajectoryLogger>();
                    spawn_logger.trajectory_filename = "Text/" + file.Name.Replace(".json", "");
                    spawn_logger.SetJsonFile();
                    spawn_text = spawn_vehicle.GetComponentInChildren<Text>();
                    group_number = Regex.Match(file.Name, @"-?\d+").Value;
                    spawn_text.text = group_number;
                }
            }
        }
    }
}