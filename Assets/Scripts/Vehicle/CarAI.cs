using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Linq;
using dubins;
using UnityEngine.UIElements;
using aStar;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
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
        public float nodeDistThreshold = 0.3f;
        public int numberSubSamplingNodes = 5;

        Rigidbody my_rigidbody;

        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        private List<AStarNode> nodePath = new();
        private int currentNodeIdx;
        private float integral = 0f;

        private HybridAStarGenerator pathFinder = null;
        private DubinsGeneratePaths dubinsPathGenerator;
        public bool drawDebug = false;

        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            m_Car = GetComponent<CarController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;
            my_rigidbody = GetComponent<Rigidbody>();

            // Rescale grid to have square shaped grid cells with size proportional to the car length
            float gridCellSize = carCollider.transform.localScale.z * 1f;
            Vector3 gridScale = mapManager.grid.transform.localScale;

            mapManager.grid.cellSize = new Vector3(
                Mathf.Round(10 * gridCellSize / gridScale.x) / 10f, 
                Mathf.Round(10 * gridCellSize / gridScale.z) / 10f,
                Mathf.Round(10 * gridCellSize / gridScale.y) / 10f);
            mapManager.Initialize();

            // Generate path
            Vector3 localStart = mapManager.localStartPosition;
            Vector3 localGoal = mapManager.localGoalPosition;
            
            currentNodeIdx = 0;
            pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), m_Car.m_MaximumSteerAngle, carCollider);
            nodePath = pathFinder.GeneratePath(
                new Vector3(localStart.x, 0.01f, localStart.z),
                new Vector3(localGoal.x, 0.01f, localGoal.z),
                transform.eulerAngles.y);
            nodePath = pathFinder.SubSamplePath(nodePath, numberSubSamplingNodes);

            // Draw unsmoothed path
            Vector3 old_wp = localStart;
            foreach (var wp in nodePath)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp.LocalPosition), Color.magenta, 1000f);
                old_wp = wp.LocalPosition;
            }

            // dubinsPathGenerator = new DubinsGeneratePaths();
            // path = GenerateSmoothedPath();

            // // Draw smoothed path
            // old_wp = localStart;
            // foreach (var wp in path)
            // {
            //     Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
            //     old_wp = wp;
            // }
            
        }

        // private List<Vector3> GenerateSmoothedPath()
        // {
        //     // Generate smooth path by creating a dubins path between all path nodes 

        //     var smoothedPath = new List<Vector3>();
        //     for (int i = 0; i < path.Count-2; ++i)
        //     {
        //         Vector3 currentNode = mapManager.grid.LocalToWorld(path[i]);
        //         Vector3 nextNode = mapManager.grid.LocalToWorld(path[i+1]);
        //         Vector3 nextNextNode = mapManager.grid.LocalToWorld(path[i+2]);
                
        //         Vector3 startDir = nextNode - currentNode;
        //         Vector3 goalDir = nextNextNode - nextNode;

        //         Quaternion startRotation = Quaternion.LookRotation(startDir);
        //         Quaternion goalRotation = Quaternion.LookRotation(goalDir);

        //         // Convert quaternions to Euler angles
        //         float startHeading = startRotation.eulerAngles.y * Mathf.Deg2Rad;
        //         float goalHeading = goalRotation.eulerAngles.y * Mathf.Deg2Rad;

        //         List<DubinsPath> pathDataList = dubinsPathGenerator.GetAllDubinsPaths(
        //             currentNode,
        //             startHeading,
        //             nextNode,
        //             goalHeading);

        //         if (pathDataList.Count > 0)
        //         {
        //             // TODO: Check for collisions and choose path that does not collide
        //             DubinsPath pathData = pathDataList[0];
        //             List<Vector3> pathCoordinates = pathData.pathCoordinates;

        //             foreach (Vector3 pathCoord in pathCoordinates)
        //             {
        //                 smoothedPath.Add(mapManager.grid.WorldToLocal(pathCoord));
        //             }
        //         }
        //     }
        //     smoothedPath.Add(path[path.Count-1]);
        //     return smoothedPath; // smoothed path with local coordinates
        // }

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

            // Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);
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
            accAngle = m_Car.MaxSpeed * (Mathf.Clamp(accAngle, 0f, 180f * lookAHead * numberSubSamplingNodes) / (180f * lookAHead * numberSubSamplingNodes));

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                targetPosition = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else 
            {   // Make target velocity lower in curves
                Vector3 headingDir = (nextTargetPosition - targetPosition).normalized;
                Debug.Log("Heading: " + headingDir + " target speed: " + m_Car.MaxSpeed / (1 + accAngle) + " current speed: " + my_rigidbody.velocity.magnitude);
                target_velocity = m_Car.MaxSpeed * headingDir / (1 + accAngle);
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
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red, Time.deltaTime * 2);
            Debug.DrawLine(my_rigidbody.position, my_rigidbody.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(targetPosition, targetPosition + desired_acceleration, Color.black);            

            m_Car.Move(steering, acceleration, acceleration, 0f);
        }

        void OnDrawGizmos()
        {
            if (drawDebug && pathFinder != null && pathFinder.flowField != null)
            {
                foreach (var posEntity in pathFinder.flowField)
                {
                    var cell = new Vector3Int(posEntity.Key.x, posEntity.Key.y, 1);
                    var cellGlobal = mapManager.grid.CellToWorld(cell);

                    var gizmoSize = mapManager.grid.cellSize;
                    gizmoSize.y = 0.005f;
                    gizmoSize.Scale(mapManager.grid.transform.localScale * 0.8f);
                    
                    Gizmos.color = Mathf.CorrelatedColorTemperatureToRGB(1000f + 1000f * posEntity.Value);
                    Gizmos.DrawCube(cellGlobal, gizmoSize);
                }
            }
        }
    }

}