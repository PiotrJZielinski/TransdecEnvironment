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

    float angle;
    Vector3 pos;
    bool collided;
    bool missed;
    bool success;

    Vector3 maxVelocity;
    float maxYawVelocity;

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
        // calculate max velocity with set parameters
        maxVelocity = new Vector3(engine.maxForceLateral / (rbody.drag * rbody.mass), 
                                  engine.maxForceVertical / (rbody.drag * rbody.mass),
                                  engine.maxForceLongitudinal / (rbody.drag * rbody.mass));
        maxYawVelocity = engine.maxTorqueYaw / (rbody.inertiaTensor.y * rbody.angularDrag);
    }

    public override void AgentReset() {
        this.rbody.angularVelocity = Vector3.zero;
        this.rbody.velocity = Vector3.zero;
        initializer.PutAll(randomQuarter, randomPosition, randomOrientation);
        targetCenter = GetComplexBounds(target).center + targetOffset;
        targetRotation = target.GetComponent<Rigidbody>().rotation;
        ResetReward();
        if (dataCollection) {
            annotations.activate = true;
            agentParameters.numberOfActionsBetweenDecisions = 1;
        }
        target.SetActive(positiveExamples);
        collided = false;
        missed = false;
        success = false;
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
        float reward = CalculateReward();
        SetReward(reward);
        if (engine.isAboveSurface() || missed || success)
            Done();
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

    float VelocityReward() {
        // Reward based on comparing velocity vector with direction towards target
        Vector3 towardsTarget = Vector3.Normalize(targetCenter - rbody.position);
        float velocityAngleDiff = (float)(Math.Cos(Vector3.Angle(towardsTarget, rbody.velocity) * Math.PI / 180));
        float velocityReward = velocityAngleDiff * rbody.velocity.magnitude / maxVelocity.z;
        return velocityReward;
    }

    float Tanh2TrigonometricAngle(float tanh) {
        float tAngle = (float)(Math.PI * tanh / Math.Tanh(1.0f));
        return tAngle; 
    }

    float AngularVelocityReward() {
        // Reward based on comparing target angular velocity value with real robot rotation velocity
        Vector3 targetForward = targetRotation * Vector3.forward;
        float direction = Math.Sign(targetCenter.x - rbody.position.x);
        if (Math.Sign(targetForward.x) != direction)
            targetForward = Quaternion.AngleAxis(180, Vector3.up) * targetForward;
        Vector3 agentForward = rbody.rotation * Vector3.forward;
        float yawVelocity = rbody.angularVelocity.y;
        float ySignedAngleDiff = -Vector3.SignedAngle(targetForward, agentForward, Vector3.up) / 180;
        float targetYawVelocity = (float)(Math.Tanh(ySignedAngleDiff));
        float angularVelocityReward = (float)(Math.Cos(Tanh2TrigonometricAngle(yawVelocity) - Tanh2TrigonometricAngle(targetYawVelocity)));
        return angularVelocityReward;
    }

    float ExpDiscountFactor(float val, float target = 0.1f, float at = 5000.0f) {
        // discount value using negative exponential function /(f(x)=e^{-a*x})/
        float a = (float)(Math.Log(target) / at);
        return (float)(Math.Exp(a*val));
    }

    float LinDiscountFactor(float val, float target = 0.1f, float at = 5000.0f) {
        // discount value using negative linear function /(f(x) = -a*x + b)/
        float a = (target - 1) / at;
        float b = 1.0f;
        return a * val + b;
    }

    Vector3 PositionReward() {
        // Reward based on position relative to target
        Vector3 relPosition = GetPosition();
        relPosition.x = ExpDiscountFactor(relPosition.x, target: 0.3f, at: 0.8f);
        relPosition.y = ExpDiscountFactor(relPosition.y, target: 0.3f, at: 0.5f);
        relPosition.z = ExpDiscountFactor(relPosition.z, target: 0.3f, at: 2.0f);
        return relPosition;
    }

    float RotationReward() {
        // Reward based on difference between facing pararelly with the target and current robot facing
        Vector3 targetForward = targetRotation * Vector3.forward;
        float direction = Math.Sign(targetCenter.x - rbody.position.x);
        if (Math.Sign(targetForward.x) != direction)
            targetForward = Quaternion.AngleAxis(180, Vector3.up) * targetForward;
        Vector3 agentForward = rbody.rotation * Vector3.forward;
        float ySignedAngleDiff = -Vector3.SignedAngle(targetForward, agentForward, Vector3.up) / 180;
        float rotationReward = (float)(Math.Cos(ySignedAngleDiff * Math.PI));
        return rotationReward;
    }

    float CalculateReward(bool terminal = false) {
        float bias = 0.2f;
        // velocity reward
        float velocityReward = VelocityReward();
        float velWeight = 0.8f;
        // angular velocity reward
        float angularVelocityReward = AngularVelocityReward();
        float angVelWeight = 0.2f;
        // current position reward
        Vector3 positionReward = PositionReward();
        float positionRewardX = positionReward.x;
        float positionRewardY = positionReward.y;
        float positionRewardZ = positionReward.z;
        float posWeight = 0.1f;
        // current location reward
        float rotationReward = RotationReward();
        float rotWeight = 0.2f;
        int currentStep = GetStepCount();
        int maxSteps = 4000;
        float reward;
        if (engine.isAboveSurface() || missed)
            reward = -10.0f;
        else if (success) {
            reward = 10.0f * rotationReward * (rotWeight +
                    posWeight * (positionRewardX + positionRewardY + positionRewardZ) / 3) *
                LinDiscountFactor(val: currentStep, target: 0.5f, at: (float)(maxSteps));
            if (reward > 0.0f)
                reward = 5 * reward;
        }
        else {
            // state reward:
            /* reward = (positionRewardZ + positionRewardX + positionRewardY + rotationReward) / 4;*/
            // action reward:
            /* reward = (velocityReward + angularVelocityReward) / 2; */ 
            // without angvel:
            /* reward = ((velocityReward + bias) / (1.0f + bias)) * (velWeight +
                         posWeight * positionRewardZ * (positionRewardX + positionRewardY) +
                         rotWeight * rotationReward) /
                     (velWeight + posWeight + rotWeight);*/
            // average:
            /* reward = (positionRewardZ + positionRewardX + positionRewardY + 
                     rotationReward + velocityReward + angularVelocityReward) / 6;*/
            //best
            reward = ((velocityReward + bias) / (1.0f + bias)) * (velWeight +
                        angVelWeight * angularVelocityReward +
                        posWeight * positionRewardZ * (positionRewardX + positionRewardY) +
                        rotWeight * rotationReward) /
                    (velWeight + angVelWeight + posWeight + rotWeight);
            // reduce with time elapsed
            if (reward > 0)
                reward = reward * ExpDiscountFactor(val: currentStep, target: 0.1f, at: (float)(maxSteps / 2));
            if (collided)
                reward -= 1.0f;
        }
        return reward;
    }

    void OnCollisionEnter() {
        collided = true;
    }

    void OnCollisionExit() {
        collided = false;
    }

    void OnTriggerEnter(Collider other) {
        if (other.gameObject.name == "TargetPlane" && targetReset)
            success = true;
        else if (other.gameObject.name == "MissPlane" && targetReset)
            missed = true;
    }
}
