using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Accelerometer : MonoBehaviour
{
	private Rigidbody rbody;
	private GameObject robot; 
    private Vector3 lastVelocity;
    private Vector3 acceleration;
    private Vector3 lastAngularVelocity;
    private Vector3 angularAcceleration;
    private Vector3 startRotation;
    System.DateTime startTime;
    System.DateTime startAngularTime;

    void Start()
    {
		robot = this.transform.parent.gameObject;
		rbody = robot.GetComponent<Rigidbody>();
        lastVelocity = transform.InverseTransformDirection(rbody.velocity);
        lastAngularVelocity = transform.InverseTransformDirection(rbody.angularVelocity);
        startRotation = rbody.rotation.eulerAngles;
        startTime = System.DateTime.UtcNow;
        startAngularTime = System.DateTime.UtcNow;
    }

    float UpdateTime(bool angular = false) {
        float deltaTime;
        System.DateTime currentTime = System.DateTime.UtcNow;
        if (angular) {
            deltaTime = (float)(currentTime - startAngularTime).TotalMilliseconds;
            startAngularTime = currentTime;
        }
        else {
            deltaTime = (float)(currentTime - startTime).TotalMilliseconds;
            startTime = currentTime;
        }
        return deltaTime;
    }

    public float[] GetAcceleration() {
        /* get value of accelerations: lateral, vertical, longitudinal */
        float deltaTime = UpdateTime() / 1000;
        Vector3 localVelocity = transform.InverseTransformDirection(rbody.velocity);
        acceleration = (localVelocity - lastVelocity) / deltaTime;
        float[] ret = new float[3];
        ret[0] = acceleration.x;
        ret[1] = acceleration.y;
        ret[2] = acceleration.z;
        lastVelocity = localVelocity;
        return ret;
    }

    public float[] GetAngularAcceleration() {
        /* get value of angular accelerations: pitch, yaw, roll */
        float deltaTime = UpdateTime(true) / 1000;
        Vector3 localAngularVelocity = transform.InverseTransformDirection(rbody.angularVelocity);
        angularAcceleration = (localAngularVelocity - lastAngularVelocity) / deltaTime;
        float[] ret = new float[3];
        ret[0] = angularAcceleration.x;
        ret[1] = angularAcceleration.y;
        ret[2] = angularAcceleration.z;
        lastAngularVelocity = localAngularVelocity;
        return ret;
    }

    public float[] GetRotation() {
        /* get value of angular positions: pitch, yaw, roll */
        float[] ret = new float[3];
        Vector3 rotation = rbody.rotation.eulerAngles;
        ret[0] = NormalizeRotation(rotation.x, startRotation.x);
        ret[1] = NormalizeRotation(rotation.y, startRotation.y);
        ret[2] = NormalizeRotation(rotation.z, startRotation.z);
        return ret;
    }

    private float NormalizeRotation(float current, float start){
        float result = (current - start) % 360;
        if (result < 0)
            result += 360;
        result = (result + 180) % 360 - 180;
        return result;
    }
}
