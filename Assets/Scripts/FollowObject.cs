using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

public class FollowObject : MonoBehaviour
{
    public Transform target_object;


    public Vector3 cameraPosition;
    public Vector3 cameraTargetOffset;

    public bool CameraFixed;


    public void Start()
    {
        if (target_object != null)
        {
            transform.position = target_object.position
                                 + cameraPosition.x * target_object.right
                                 + cameraPosition.z * target_object.forward
                                 + cameraPosition.y * target_object.up;

            transform.rotation = Quaternion.LookRotation(target_object.transform.position + cameraTargetOffset - transform.position);
        }
    }

    void FixedUpdate()
    {
        if (!CameraFixed)
        {
            transform.rotation = Quaternion.LookRotation(target_object.transform.position + cameraTargetOffset - transform.position);
            transform.position = target_object.position
                                 + cameraPosition.x * target_object.right
                                 + cameraPosition.z * target_object.forward
                                 + cameraPosition.y * target_object.up;
        }
        else
        {
            transform.position = target_object.position
                                 + cameraPosition.x * Vector3.right
                                 + cameraPosition.z * Vector3.forward
                                 + cameraPosition.y * Vector3.up;
        }
    }
}