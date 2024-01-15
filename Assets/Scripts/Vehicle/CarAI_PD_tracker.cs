using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI_PD_tracker : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        public GameObject my_target;

        public bool driveInCircle = false;
        public float circleRadius = 15f;
        public float circleSpeed = 5f;
        float alpha = 0f;
        public Vector3 circleCenter = Vector3.zero;
        

        public Vector3 target_velocity;
        Vector3 old_target_pos;
        Vector3 desired_velocity;

        public float k_p = 2f;
        public float k_d = 0.5f;

        Rigidbody my_rigidbody;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();

            my_rigidbody = GetComponent<Rigidbody>();

            if (my_target == null) // no target, only circle option works
            {
                old_target_pos = transform.position;
            }
            else
            {
                old_target_pos = my_target.transform.position;
            }

            

            // Initialize circle at starting position
            circleCenter = transform.position;

            // Plan your path here
            // ...
        }


        private void FixedUpdate()
        {
            Vector3 target_position;

            

            if (driveInCircle) // for the circle option
            {
                alpha +=  Time.deltaTime * (circleSpeed / circleRadius);
                target_position = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else // if target is a game object
            {
                target_position = my_target.transform.position;
                target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            }

            old_target_pos = target_position;

            // a PD-controller to get desired velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
        }
    }
}
