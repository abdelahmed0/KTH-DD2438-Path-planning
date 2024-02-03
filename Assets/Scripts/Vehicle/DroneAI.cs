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
    public float nodeDistThreshold = 0.4f;
    public int numberSubSamplingNodes = 5;
    private DroneController m_Drone; // the controller we want to use
    private MapManager mapManager;
    private BoxCollider droneCollider;
    Rigidbody my_rigidbody;
    private Vector3 oldTargetPosition;

    private HybridAStarGenerator pathFinder = null;
    private List<AStarNode> nodePath = new();
    private int currentNodeIdx;

    private void Start()
    {
        Debug.Log("asdflkjasdf");
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
        oldTargetPosition = transform.position;
        
        currentNodeIdx = 0;
        pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), 20f, droneCollider);
        nodePath = pathFinder.GeneratePath(
            new Vector3(localStart.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localStart.z),
            new Vector3(localGoal.x, mapManager.grid.WorldToLocal( droneCollider.transform.position).y, localGoal.z),
            transform.eulerAngles.y);
        
        nodePath = pathFinder.SubSamplePath(nodePath, numberSubSamplingNodes);

        // Draw unsmoothed path
        Vector3 old_wp = localStart;
        foreach (var wp in nodePath)
        {
            Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp.LocalPosition), Color.magenta, 1000f);
            old_wp = wp.LocalPosition;
        }
    }


    private void FixedUpdate()
    {
        // TODO: Implement backing up of car if stuck (velocity near zero and/or colliding with object)
        if (nodePath.Count == 0)
        {
            return;
        }

        var globalPosition = transform.position;
        Vector3 localPosition = mapManager.grid.WorldToLocal(transform.position);
        
        Vector3 localNextNode = nodePath[currentNodeIdx].LocalPosition;
        Vector3 globalNextNode = mapManager.grid.LocalToWorld(localNextNode);


        PidControllTowardsPosition();

        // If car is at pathNode, update pathNodeEnumerator
        if (currentNodeIdx < nodePath.Count - 1 && Vector3.Distance(mapManager.grid.WorldToLocal(globalPosition), localNextNode) < nodeDistThreshold)
        {
            currentNodeIdx++;
        }

        Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);
    }

    private void PidControllTowardsPosition()
        {
            AStarNode target = nodePath[currentNodeIdx];
            AStarNode nextTarget = nodePath[Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1)];
            Vector3 targetPosition = mapManager.grid.LocalToWorld(target.LocalPosition);
            Vector3 nextTargetPosition = mapManager.grid.LocalToWorld(nextTarget.LocalPosition);

            int lookAHead = 5;
            float accAngle = 0f;
            for (int i = 0; i < lookAHead * numberSubSamplingNodes; ++i)
            {
                accAngle += Mathf.Abs(Mathf.DeltaAngle(target.angle * Mathf.Rad2Deg,
                                        nodePath[Math.Clamp(currentNodeIdx+1+i, 0, nodePath.Count-1)].angle * Mathf.Rad2Deg));
            }
            accAngle = m_Drone.max_speed * (Mathf.Clamp(accAngle, 0f, 180f * lookAHead * numberSubSamplingNodes) / (180f * lookAHead * numberSubSamplingNodes));

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                targetPosition = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else 
            {   // Make target velocity lower in curves
                Vector3 headingDir = (nextTargetPosition - targetPosition).normalized;
                Debug.Log("Heading: " + headingDir + " target speed: " + m_Drone.max_speed / (1 + accAngle) + " current speed: " + my_rigidbody.velocity.magnitude);
                target_velocity = m_Drone.max_speed * headingDir / Mathf.Exp(accAngle);
            }

            // a PD-controller to get desired acceleration from errors in position and velocity
            Vector3 positionError = targetPosition - transform.position;
            Vector3 velocityError = target_velocity - my_rigidbody.velocity;
            
            Vector3 proportional = k_p * positionError;
            integral += Time.fixedDeltaTime * positionError.magnitude;
            Vector3 integralTerm = k_i * integral * positionError.normalized;
            Vector3 derivativeTerm = k_d * velocityError;

            Vector3 desired_acceleration = proportional + integralTerm + derivativeTerm;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward) / Mathf.Exp(accAngle);

            Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red, Time.deltaTime * 2);
            Debug.DrawLine(my_rigidbody.position, my_rigidbody.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(targetPosition, targetPosition + desired_acceleration, Color.black);   

            m_Drone.Move(0.1f * steering, acceleration);
        }
}