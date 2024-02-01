using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using aStar;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    public bool driveInCircle = false;
    public float circleRadius = 15f;
    public float circleSpeed = 5f;
    float alpha = 0f;
    public Vector3 circleCenter = Vector3.zero;
    private Vector3 target_velocity;
    public float k_p = 2f;
    public float k_i = 0.1f;
    public float k_d = 0.5f;
    private float integral = 0f;
    public float nodeDistThreshold = 0.6f;
    private DroneController m_Drone; // the controller we want to use
    private MapManager mapManager;
    private BoxCollider droneCollider;
    Rigidbody my_rigidbody;
    private Vector3 oldTargetPosition;

    private HybridAStarGenerator pathFinder = null;
    private List<Vector3> path = new();
    private int currentNodeIdx;

    private void Start()
    {
        droneCollider = gameObject.GetComponent<BoxCollider>();
        my_rigidbody = GetComponent<Rigidbody>();

        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        mapManager = FindObjectOfType<GameManager>().mapManager;
      
        Vector3 localStart = mapManager.localStartPosition;
        Vector3 localGoal = mapManager.localGoalPosition;
        oldTargetPosition = transform.position;
        
        currentNodeIdx = 0;
        pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), 20f, droneCollider);
        List<AStarNode> nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localStart.z),
            new Vector3(localGoal.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localGoal.z),
            transform.eulerAngles.y);
        
        foreach (var node in nodePath)
        {
            path.Add(node.LocalPosition);
        }

        // Draw unsmoothed path
        Vector3 old_wp = localStart;
        foreach (var wp in path)
        {
            Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.magenta, 1000f);
            old_wp = wp;
        }
    }


    private void FixedUpdate()
    {
        // TODO: Implement backing up of car if stuck (velocity near zero and/or colliding with object)
        if (path.Count == 0)
        {
            return;
        }

        var globalPosition = transform.position;
        Vector3 localPosition = mapManager.grid.WorldToLocal(transform.position);
        
        Vector3 localNextNode = path[currentNodeIdx];
        Vector3 globalNextNode = mapManager.grid.LocalToWorld(localNextNode);


        PidControllTowardsPosition(globalNextNode);
        // If car is at pathNode, update pathNodeEnumerator
        if (currentNodeIdx < path.Count - 1 && Vector3.Distance(mapManager.grid.WorldToLocal(globalPosition), localNextNode) < nodeDistThreshold)
        {
            currentNodeIdx++;
        }

        Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);
    }

    private void PidControllTowardsPosition(Vector3 globalTargetPosition)
        {
            // TODO: add function based on node angle to decide velocity
            Vector3 targetPosition;

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                targetPosition = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else // target is next node in path
            {
                targetPosition = globalTargetPosition;
                target_velocity = (targetPosition - oldTargetPosition) / Time.fixedDeltaTime;
            }

            oldTargetPosition = targetPosition;

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

            Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red);
            Debug.DrawLine(oldTargetPosition, oldTargetPosition + my_rigidbody.velocity, Color.magenta);
            Debug.DrawLine(oldTargetPosition, oldTargetPosition + desired_acceleration, Color.black);

            m_Drone.Move(0.1f * steering, acceleration);
        }
}