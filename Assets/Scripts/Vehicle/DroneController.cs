using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;


public class DroneController : MonoBehaviour
{
    public float max_speed = 15f;
    public float max_acceleration = 15f;

    private float v = 0f; //desired acceleration first component
    private float h = 0f; //desired acceleration second component
    private List<Transform> propellers;

    public void Move(float h_in, float v_in)
    {
        h = h_in;
        v = v_in;
    }

    // Start is called before the first frame update
    void Start()
    {
        propellers = new List<Transform>();
        foreach (Transform child in transform.Find("droneModel"))
        {
            if (child.name == "Propeller")
            {
                propellers.Add(child.Find("Rotation"));
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
    }

    private void FixedUpdate()
    {
        var rigidBody = GetComponent<Rigidbody>();

        var acceleration = (Vector3.right * h + Vector3.forward * v) * max_acceleration;
        if (acceleration.magnitude > max_acceleration)
        {
            acceleration = acceleration.normalized * max_acceleration;
        }

        rigidBody.AddForce(acceleration, ForceMode.Acceleration);

        if (rigidBody.velocity.magnitude > max_speed)
        {
            rigidBody.velocity = rigidBody.velocity.normalized * max_speed;
        }


        var targetRotation = Quaternion.LookRotation(
            Vector3.forward * 40 - new Vector3(0, acceleration.z, 0),
            Vector3.up * 40 + new Vector3(acceleration.x, 0, 0));
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 10.0f * Time.fixedDeltaTime);

        foreach (Transform propeller in propellers)
        {
            propeller.Rotate(new Vector3(0, 67, 0));
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Entered collision with " + collision.gameObject.name);
        max_acceleration /= 2f;
    }
}