using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.IO;
using PathPlanning;
using util;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        // public readonly float CELLSIZE = 0.25f;

        private List<Vector2> positions;
        private List<Vector2> velocities;
        private List<float> times;

        private Vector2 prevPos;

        public readonly float k_p = 2f;
        public readonly float k_d = 1.5f;

        public GameObject my_target;

        private int target_pos_vel_idx = 0;
        private CollisionDetector detector;

        private Vector3 target_position;
        private Vector3 target_velocity;
        private Vector3 old_target_pos;
        private float MARGIN = 2.5f;
        private float startTime;
        private void Start()
        {   

            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            m_Car = GetComponent<CarController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;

            var sw = System.Diagnostics.Stopwatch.StartNew();
            detector = new CollisionDetector(mapManager, margin: 2.5f);
            sw.Stop();
            Debug.Log($"Detector: {sw.ElapsedMilliseconds} ms");

            RRT rrt = new();

            // Set the dynamic constraints
            rrt.localPlanner.acceleration = 6f;
            rrt.localPlanner.deceleration = 6f;
            rrt.localPlanner.minSpeed = 10f;
            rrt.localPlanner.maxSpeed = 75f;
            rrt.localPlanner.minTurnRadius = 6f;
            rrt.localPlanner.turnRadiusUncap = 35f;

            // Set RRT settings
            rrt.areaSearch = true;
            rrt.pruning = false;
            rrt.startWithKnn = true;
            rrt.goalPathBias = true;
            rrt.biasProbability = 0.5f;
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
            //rrt.DebugDrawTree();
        }

        private void OnDrawGizmos()
        {
            detector.DebugDrawBoundingBoxes();
            //detector.DebugDrawGrid();
        }
        private Vector2 Vector3ToVector2(Vector3 v)
        {
            return new Vector2(v.x, v.z);
        }
        private Vector3 Vector2ToVector3(Vector2 v)
        {
            return new Vector3(v.x, 0.4f, v.y);
        }

        private void FixedUpdate()
        {
            ControlCar();   
        }

        // Tracking
        private void ControlCar()
        {
            Vector2 carPos = new Vector2(transform.position.x, transform.position.z);
            Vector2 carVelocity = (carPos - prevPos) / Time.fixedDeltaTime;
            float carSpeed = carVelocity.magnitude;
            prevPos = carPos;

            // Look ahead proportional to speed
            float baseLookAhead = 3.5f;
            float referenceSpeed = 9f;
            float lookAhead = baseLookAhead * Mathf.Max(carSpeed / referenceSpeed, 0.5f);
            
            // Get position and velocity that far ahead on the path
            Vector2 lookAheadPos, lookAheadVel;
            (lookAheadPos, lookAheadVel) = TrackingUtil.LookAheadPositionAndVelocity(carPos, lookAhead, positions, times);

            float futureSpeed = lookAheadVel.magnitude;
            
            // Calculate inputs
            float maxSteerAngle = 30f; // Lower = more responsive, higher = more smooth but may overshoot
            float maxAcceleration = 2f; // Same as above
            float angle = Vector2.SignedAngle(lookAheadPos-carPos, new Vector2(transform.forward.x, transform.forward.z));
            float steering = Mathf.Clamp(angle / maxSteerAngle, -1, 1);
            float acceleration = TrackingUtil.CalculateAcceleration(maxAcceleration, carPos, lookAheadPos, carSpeed, futureSpeed);

            Debug.DrawLine(Vector2ToVector3(carPos), Vector2ToVector3(carPos) + new Vector3(0, carSpeed, 0), Color.red);
            Debug.DrawLine(Vector2ToVector3(lookAheadPos), Vector2ToVector3(lookAheadPos) + new Vector3(0, futureSpeed, 0), Color.red);
            Debug.DrawLine(Vector2ToVector3(carPos), Vector2ToVector3(lookAheadPos), Color.magenta);
            
            m_Car.Move(steering, acceleration, acceleration > 0 ? 0 : -acceleration, 0f);
        }
    }
}