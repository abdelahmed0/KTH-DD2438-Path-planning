using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.IO;
using PathPlanning;
using util;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    private DroneController m_Drone; // the controller we want to use
    private MapManager mapManager;
    private BoxCollider droneCollider;

    private List<Vector2> positions;
    private List<Vector2> velocities;
    private List<float> times;

    public readonly float k_p = 2.5f;
    public readonly float k_d = 1.0f;

    public GameObject my_target;

    private int target_pos_vel_idx = 0;
    private CollisionDetector detector;

    private Vector3 target_position;
    private Vector3 target_velocity;
    private Vector3 old_target_pos;
    private float MARGIN = 1.5f;
    private float startTime;

    private Vector2 prevPos;

    private void Start()
    {   
        // Empty invisible game object to follow
        my_target = new GameObject();

        droneCollider = gameObject.GetComponent<BoxCollider>();

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        mapManager = FindObjectOfType<GameManager>().mapManager;

        var sw = System.Diagnostics.Stopwatch.StartNew();
        detector = new CollisionDetector(mapManager, margin: 2.5f, yMin: -0.1f, yMax: 5f);
        sw.Stop();
        Debug.Log($"Detector: {sw.ElapsedMilliseconds} ms");

        RRT rrt = new RRT();

        // Set the dynamic constraints
        rrt.localPlanner.acceleration = 15f;
        rrt.localPlanner.deceleration = 15f;
        rrt.localPlanner.minSpeed = 6f;
        rrt.localPlanner.maxSpeed = 15f;
        rrt.localPlanner.minTurnRadius = 6f;
        rrt.localPlanner.turnRadiusUncap = 1000f;

        // Set RRT settings
        rrt.areaSearch = true;
        rrt.pruning = true;
        rrt.startWithKnn = true;
        float timeLimit = 60f; // Stop searching after a certain time

        sw.Restart();
        (positions, times) = rrt.FindPath(mapManager, detector, timeLimit); // List<Vector2> of positions, List<float> of timestamps
        sw.Stop();
        Debug.Log($"RRT: {sw.ElapsedMilliseconds} ms");

        prevPos = Vector3ToVector2(mapManager.GetGlobalStartPosition());

        // Save statistics
        string fileName = "cost_history.json";
        string jsonString = JsonUtility.ToJson(rrt.costHistory);
        File.WriteAllText(fileName, jsonString);

        // Draw path
        rrt.DebugDrawPath();
        // rrt.DebugDrawTree();
    }

    private void OnDrawGizmos()
    {
        detector.DebugDrawBoundingBoxes();
    }
    private Vector2 Vector3ToVector2(Vector3 v)
    {
        return new Vector2(v.x, v.z);
    }
    private Vector3 Vector2ToVector3(Vector2 v)
    {
        return new Vector3(v.x, 0.5f, v.y);
    }

    private void FixedUpdate()
    {
        // this is how you control the car
        /*target_position = my_target.transform.position;
        target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
        old_target_pos = target_position;
        var (currentPos, currentVel) = TrackingUtil.GetPositionAndVelocity(Math.Max(Time.realtimeSinceStartup - startTime, 0), positions, times);
        my_target.transform.position = Vector2ToVector3(currentPos);
        Debug.DrawLine(my_target.transform.position, my_target.transform.position + Vector3.up * 10f, Color.black);    

        // a PD-controller to get desired acceleration from errors in position and velocity
        Vector3 position_error = target_position - transform.position;
        Vector3 velocity_error = target_velocity - GetComponent<Rigidbody>().velocity;
        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

        Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        Debug.DrawLine(transform.position, transform.position + GetComponent<Rigidbody>().velocity, Color.blue);
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

        // this is how you control the car
        Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        m_Drone.Move(steering, acceleration);*/

        ControlDrone();
    }

    // Tracking
    private void ControlDrone()
    {
        Vector2 currentPos = new Vector2(transform.position.x, transform.position.z);
        Vector2 currentVelocity = (currentPos - prevPos) / Time.fixedDeltaTime;
        float speed = currentVelocity.magnitude;
        prevPos = currentPos;

        // Look ahead proportional to speed
        float baseLookAhead = 5f;
        float referenceSpeed = 10f;
        float lookAhead = baseLookAhead * Mathf.Max(speed / referenceSpeed, 0.5f);

        // Get position and velocity that far ahead on the path
        Vector2 lookAheadPos, lookAheadVel;
        (lookAheadPos, lookAheadVel) = TrackingUtil.LookAheadPositionAndVelocity(currentPos, lookAhead, positions, times);

        // Calculate parallel and transverse velocity components wrt the direction of the look ahead position
        Vector2 lookDir = (lookAheadPos - currentPos).normalized;
        float velocityParallel = Vector2.Dot(currentVelocity, lookDir);
        Vector2 velocityTransverse = currentVelocity - velocityParallel * lookDir; // Use this to kill velocity in any direction other than the one we want to go

        // Calculate inputs
        float ap = TrackingUtil.CalculateAccelerationUnclamped(currentPos, lookAheadPos, velocityParallel, lookAheadVel.magnitude);
        float balanceMultiplier = 2f;
        Vector2 accel = ap * lookDir - velocityTransverse * balanceMultiplier;

        Debug.DrawLine(Vector2ToVector3(currentPos), Vector2ToVector3(lookAheadPos), Color.magenta);
        m_Drone.Move(accel.x, accel.y);
    }
}
