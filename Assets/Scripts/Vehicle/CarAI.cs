using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Linq;
using UnityEngine.UIElements;
using aStar;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        public float colliderResizeFactor = 2f;
        public int numberSteeringAngles = 3;        
        private Vector3 target_velocity;
        public float k_p = 2f;
        public float k_i = 0.1f;
        public float k_d = 0.7f;
        public float nodeDistThreshold = 0.4f;

        Rigidbody my_rigidbody;

        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;
        private List<AStarNode> nodePath = new();
        private int currentNodeIdx;
        private float integral = 0f;

        private HybridAStarGenerator pathFinder = null;
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
            pathFinder = new(mapManager.grid, mapManager.GetObstacleMap(), m_Car.m_MaximumSteerAngle, carCollider, colliderResizeFactor, false);
            nodePath = pathFinder.GeneratePath(
                new Vector3(localStart.x, 0.01f, localStart.z),
                new Vector3(localGoal.x, 0.01f, localGoal.z),
                transform.eulerAngles.y,
                numberSteeringAngles);

            nodePath = pathFinder.SmoothPath(nodePath);

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
                    || Vector3.Distance(localPosition, localNextNextNode) <  Vector3.Distance(localPosition, localNextNode)))
            {
                currentNodeIdx++;
            }

            // // TODO: Implement backing up of car if stuck (velocity near zero and/or colliding with object)
            // if (my_rigidbody.velocity.magnitude < 1f)
            // {
            //     Dictionary<Vector3, bool> free = new Dictionary<Vector3, bool>
            //     {
            //         { Vector3.forward, true },
            //         { Vector3.right, true },  
            //         { Vector3.back, true },   
            //         { Vector3.left, true }    
            //     };
            //     foreach (Vector3 dir in free.Keys)
            //     {
            //          free[dir] = !Physics.Raycast(transform.position,
            //                             dir,
            //                             out var hitInfo,
            //                             (carCollider.ClosestPointOnBounds(dir * float.MaxValue) - transform.position).magnitude + 1f);
            //     }
            // }

            PidControllTowardsPosition();
        }

        private void PidControllTowardsPosition()
        {
            AStarNode target = nodePath[currentNodeIdx];
            AStarNode nextTarget = nodePath[Math.Clamp(currentNodeIdx+1, 0, nodePath.Count-1)];
            Vector3 targetPosition = mapManager.grid.LocalToWorld(target.LocalPosition);
            Vector3 nextTargetPosition = mapManager.grid.LocalToWorld(nextTarget.LocalPosition);
  
            // Make target velocity lower in curves, where points are denser
            target_velocity = nextTargetPosition - targetPosition;;

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