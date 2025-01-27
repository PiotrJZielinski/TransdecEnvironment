﻿using MLAgents;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RobotAcademy : Academy { 
    public enum RobotControl {
        player, python
    }

    public enum DataCollection {
        gate, path
    }

    [Header("Controller settings")]
    public RobotControl control;
    public Brain learningBrain;
    public Brain playerBrain;

    [Header("Start position settings")]
    public bool randomQuarter = true;
    public bool randomPosition = true;
    public bool randomOrientation = true;


    [Header("Data collection settings")]
    public DataCollection mode;
    public GameObject gateTargetObject;
    public GameObject pathTargetObject;

    [Header("Debug settings - use carefully!")]
    public bool forceDataCollection = false;
    public bool forceNoise = false;
    public bool forceNegativeExamples = false;

    RobotAgent robot;

    void OnValidate() {
        robot = GameObject.Find("Robot").GetComponent<RobotAgent>();
        if (control == RobotControl.player) {
            robot.GiveBrain(playerBrain);
            broadcastHub.broadcastingBrains.Clear();
            broadcastHub.broadcastingBrains.Add(playerBrain);
        }
        else {
            robot.GiveBrain(learningBrain);
            broadcastHub.broadcastingBrains.Clear();
            broadcastHub.broadcastingBrains.Add(learningBrain);
            broadcastHub.SetControlled(learningBrain, true);
        }
        if (resetParameters["CollectData"] == 1 || forceDataCollection) {
            robot.sendRelativeData = true;
            robot.dataCollection = true;
            robot.mode = mode;
            robot.gateTargetObject = gateTargetObject;
            robot.pathTargetObject = pathTargetObject;
        }
        else {
            robot.sendRelativeData = false;
            robot.dataCollection = false;
        }
        if (resetParameters["Positive"] == 0 || forceNegativeExamples)
            robot.positiveExamples = false;
        else
            robot.positiveExamples = true;
        robot.randomQuarter = randomQuarter;
        robot.randomPosition = randomPosition;
        robot.randomOrientation = randomOrientation;
    }

    public override void AcademyReset() {
        if (resetParameters["CollectData"] == 1 || forceDataCollection)
        {
            robot.sendRelativeData = true;
            robot.dataCollection = true;
            robot.mode = mode;
            robot.gateTargetObject = gateTargetObject;
            robot.pathTargetObject = pathTargetObject;
            if (resetParameters["EnableNoise"] == 1 || forceNoise)
                robot.addNoise = true;
            else
                robot.addNoise = false;
        }
        else
        {
            robot.sendRelativeData = false;
            robot.dataCollection = false;
        }
        if (resetParameters["Positive"] == 0 || forceNegativeExamples)
            robot.positiveExamples = false;
        else
            robot.positiveExamples = true;
    }

    public override void InitializeAcademy() {
        robot = GameObject.Find("Robot").GetComponent<RobotAgent>();
        robot.agentParameters.maxStep = (int)(resetParameters["AgentMaxSteps"]);
        if (resetParameters["CollectData"] == 1 || forceDataCollection) {
            if (resetParameters["EnableNoise"] == 1 || forceNoise)
                robot.addNoise = true;
            else
                robot.addNoise = false;
        }
    }
}
