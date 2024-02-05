using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using aStar;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    public float colliderResizeFactor = 1.5f;
    public int numberSteeringAngles = 3;
    public bool smoothPath = false;
    public bool allowReversing = false;
    private Vector3 target_velocity;
    public float k_p = 2f;
    public float k_i = 0f;
    public float k_d = 0.8f;
    private float integral = 0f;
    public float nodeDistThreshold = 0.1f;
    private DroneController m_Drone; // the controller we want to use
    private MapManager mapManager;
    private BoxCollider droneCollider;
    Rigidbody my_rigidbody;

    private HybridAStarGenerator pathFinder = null;
    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;

    private void Start()
    {
        droneCollider = gameObject.GetComponent<BoxCollider>();
        my_rigidbody = GetComponent<Rigidbody>();
        m_Drone = GetComponent<DroneController>();
        mapManager = FindObjectOfType<GameManager>().mapManager;

         // Rescale grid to have square shaped grid cells with size proportional to the drone's length
        float gridCellSize = droneCollider.transform.localScale.z * 1f;
        Vector3 gridScale = mapManager.grid.transform.localScale;

        mapManager.grid.cellSize = new Vector3(
            Mathf.Round(10 * gridCellSize / gridScale.x) / 10f, 
            Mathf.Round(10 * gridCellSize / gridScale.z) / 10f,
            Mathf.Round(10 * gridCellSize / gridScale.y) / 10f);
        mapManager.Initialize();

      
        Vector3 localStart = mapManager.localStartPosition;
        Vector3 localGoal = mapManager.localGoalPosition;
        
        currentNodeIdx = 0;
        pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), 40f, droneCollider, colliderResizeFactor, true, allowReversing, 0f);
        nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localStart.z),
            new Vector3(localGoal.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localGoal.z),
            transform.eulerAngles.y,
            numberSteeringAngles);
        
        nodePath = smoothPath ? pathFinder.SmoothPath(nodePath) : nodePath;

        Vector3 old_wp = localStart;
        foreach (var wp in nodePath)
        {
            Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp.LocalPosition), Color.magenta, 1000f);
            old_wp = wp.LocalPosition;
        }
    }

    private void FixedUpdate()
    {
        if (nodePath.Count == 0)
        {
            return;
        }

        Vector3 localPosition = mapManager.grid.WorldToLocal(transform.position);
        Vector3 localNextNode = nodePath[currentNodeIdx].LocalPosition;
        int nextNextIndex = Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1);
        Vector3 localNextNextNode = nodePath[nextNextIndex].LocalPosition;

        if (currentNodeIdx < nodePath.Count - 1 
                && (Vector3.Distance(localPosition, localNextNode) < nodeDistThreshold 
                    || Vector3.Distance(localPosition, localNextNextNode) < Vector3.Distance(localPosition, localNextNode)))
        {
            currentNodeIdx++;
        }

        PidControllTowardsPosition();
    }

    private void PidControllTowardsPosition()
        {
            AStarNode target = nodePath[currentNodeIdx];
            AStarNode nextTarget = nodePath[Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1)];
            Vector3 targetPosition = mapManager.grid.LocalToWorld(target.LocalPosition);
            Vector3 nextTargetPosition = mapManager.grid.LocalToWorld(nextTarget.LocalPosition);

            // Make target velocity lower in curves, where points are closer together
            target_velocity = nextTargetPosition - targetPosition;

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 positionError = targetPosition - transform.position;
            Vector3 velocityError = target_velocity - my_rigidbody.velocity;
            
            Vector3 proportional = k_p * positionError;
            integral += Time.fixedDeltaTime * positionError.magnitude;
            Vector3 integralTerm = k_i * integral * positionError.normalized;
            Vector3 derivativeTerm = k_d * velocityError;

            Vector3 desired_acceleration = proportional + integralTerm + derivativeTerm;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red, Time.deltaTime * 2);
            Debug.DrawLine(my_rigidbody.position, my_rigidbody.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(targetPosition, targetPosition + desired_acceleration, Color.black);   

            m_Drone.Move(steering, acceleration);
        }
}