using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpdateRobot : MonoBehaviour
{
    [SerializeField]
    private GameObject subscriber;
    RobotManager robotManager;
    JointSubscriber jointSubscriber;


    void Awake()
    {
        robotManager = GetComponent<RobotManager>();
        jointSubscriber = subscriber.GetComponent<JointSubscriber>();
    }

    // Update is called once per frame
    void Update()
    {
        robotManager.SetJointAngles(jointSubscriber.jointStatesDeg);
    }
}
