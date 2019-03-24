using UnityEngine;
using MLAgents;
using System;

public class RobotAgent : Agent {

    [Header("Reward function settings")]
    public GameObject target;
    public Vector3 targetOffset = Vector3.zero;

    [Header("Additional settings")]
    public bool sendRelativeData = false;
    public bool dataCollection = false;
    public bool addNoise = false;
    public bool positiveExamples = true;
    public bool targetReset = false;
    public RobotAcademy.DataCollection mode;
    public GameObject gateTargetObject;
    public GameObject pathTargetObject;
    public bool randomQuarter = true;
    public bool randomPosition = true;
    public bool randomOrientation = true;

    Rigidbody rbody;
    Engine engine;
    Accelerometer accelerometer;
    DepthSensor depthSensor;
    Vector3 targetCenter;
    Quaternion targetRotation;
    Vector3 startPos;
    float startAngle;

    float angle;
    Vector3 pos;

    TargetAnnotation annotations;
    RandomInit initializer;
    RandomPosition positionDrawer;

    void OnValidate() {
        GameObject agent = transform.parent.gameObject;
        annotations = agent.GetComponent<TargetAnnotation>();
        initializer = agent.GetComponent<RandomInit>();
        positionDrawer = agent.GetComponent<RandomPosition>();
        positionDrawer.agent = transform.gameObject;
        if (mode == RobotAcademy.DataCollection.gate){
            annotations.target = gateTargetObject;
            positionDrawer.target = gateTargetObject;
        }
        else if (mode == RobotAcademy.DataCollection.path){
            annotations.target = pathTargetObject;
            positionDrawer.target = pathTargetObject;
        }
    }

	void Start () {
        GameObject agent = transform.parent.gameObject;
        annotations = agent.GetComponent<TargetAnnotation>();
        initializer = agent.GetComponent<RandomInit>();
        positionDrawer = agent.GetComponent<RandomPosition>();
        positionDrawer.agent = transform.gameObject;
        rbody = GetComponent<Rigidbody>();
        engine = transform.Find("Engine").GetComponent<Engine>();
        accelerometer = transform.Find("Accelerometer").GetComponent<Accelerometer>();
        depthSensor = transform.Find("DepthSensor").GetComponent<DepthSensor>();
    }

    public override void AgentReset() {
        this.rbody.angularVelocity = Vector3.zero;
        this.rbody.velocity = Vector3.zero;
        initializer.PutAll(randomQuarter, randomPosition, randomOrientation);
        targetCenter = GetComplexBounds(target).center;
        targetRotation = target.GetComponent<Rigidbody>().rotation;
        startPos = GetPosition();
        startAngle = GetAngle();
        ResetReward();
        if (dataCollection) {
            annotations.activate = true;
            agentParameters.numberOfActionsBetweenDecisions = 1;
        }
        target.SetActive(positiveExamples);
    }

    public override void CollectObservations() {
        float[] toSend = new float[19];
        float[] acceleration = accelerometer.GetAcceleration();
        float[] angularAcceleration = accelerometer.GetAngularAcceleration();
        float[] rotation = accelerometer.GetRotation();
        // acceleration data
        int toSendCell = 0;
        acceleration.CopyTo(toSend, toSendCell);
        // angular acceleration data
        toSendCell += acceleration.Length;
        angularAcceleration.CopyTo(toSend, toSendCell);
        // rotation data
        toSendCell += angularAcceleration.Length;
        rotation.CopyTo(toSend, toSendCell);
        // depth data
        toSendCell += rotation.Length;
        toSend[toSendCell] = depthSensor.GetDepth();
        // bounding box
        toSendCell += 1;
        if (dataCollection && positiveExamples)
            annotations.GetBoundingBox().CopyTo(toSend, toSendCell);
        // positive/negative example
        toSendCell += 4;
        if (positiveExamples)
            toSend[toSendCell] = 1.0f;
        else
            toSend[toSendCell] = 0.0f;
        // relative position data
        if (sendRelativeData){
            toSend[toSendCell + 1] = pos.x;
            toSend[toSendCell + 2] = pos.y;
            toSend[toSendCell + 3] = pos.z;
            toSend[toSendCell + 4] = angle;
        }
        AddVectorObs(toSend);
    }

    public override void AgentAction(float[] vectorAction, string textAction){
        if (dataCollection)
            positionDrawer.DrawPositions(addNoise, randomQuarter, randomPosition);
        else
            engine.Move(vectorAction[0], vectorAction[1], vectorAction[2], vectorAction[3]);
        pos = GetPosition();
        angle = GetAngle();
        float currentReward = CalculateReward();
        SetReward(currentReward);
        if (engine.isAboveSurface()) {
            SetReward(-1.0f);
            Done();
        }
    }

    public Bounds GetComplexBounds(GameObject obj) {
        Bounds bounds = new Bounds (obj.transform.position, Vector3.zero);
        Renderer[] renderers = obj.GetComponentsInChildren<Renderer>();
        foreach(Renderer renderer in renderers)
        {
            bounds.Encapsulate(renderer.bounds);
        }
        return bounds;
    }

    Vector3 GetPosition() {
        // relative position
        Vector3 distToCenter = target.transform.InverseTransformPoint(targetCenter);
        Vector3 relativePos = target.transform.InverseTransformPoint(rbody.position) - distToCenter - targetOffset;
        relativePos.x = Math.Abs(relativePos.x);
        relativePos.y = Math.Abs(relativePos.y);
        relativePos.z = Math.Abs(relativePos.z);
        return relativePos;
    }

    float GetAngle() {
        // relative angle
        float relativeYaw = (Quaternion.Inverse(targetRotation) * rbody.rotation).eulerAngles.y;
        relativeYaw = Math.Abs((relativeYaw + 180) % 360 - 180);
        return relativeYaw;
    }

    float CalculateReward(float headingProp = 0.5f, float velocityProp = 0.5f) {
        // normalize proportions
        headingProp = headingProp / (headingProp + velocityProp);
        velocityProp = velocityProp / (headingProp + velocityProp);
        int currentStep = GetStepCount();
        int maxSteps = 5000;
        Vector3 towardsTarget = Vector3.Normalize(targetCenter - rbody.position);
        Vector3 agentForward = rbody.rotation * Vector3.forward;
        float headingAngleDiff = (float)(Math.Cos(Vector3.Angle(towardsTarget, agentForward) * Math.PI / 180));
        float velocityAngleDiff = (float)(Math.Cos(Vector3.Angle(towardsTarget, rbody.velocity) * Math.PI / 180));
        float reward = headingProp * headingAngleDiff + velocityProp * velocityAngleDiff * rbody.velocity.magnitude / 2;
        if (reward > 0)
            reward = reward * (1 - 0.5f * currentStep / maxSteps);
        return reward;
    }

    float CalculateSingleReward(float current, float start) {
        float x0 = Math.Max(0.001f, start);
        return (float)(Math.Pow(2, (x0 - current) / x0) - 1);
    }

    void OnCollisionEnter() {
        SetReward(-1.0f);
        Done();
    }

    void OnTriggerEnter(Collider other) {
        if (other.gameObject.name == "TargetPlane" && targetReset){
            int currentStep = GetStepCount();
            int maxSteps = 5000;
            SetReward(1 - 0.5f * currentStep / maxSteps);
            Done();
        }
    }
}
