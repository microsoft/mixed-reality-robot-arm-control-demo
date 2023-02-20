using RosMessageTypes.Geometry;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;


public class TargetPosePublisher : MonoBehaviour
{

    ROSConnection ros;
    [SerializeField]
    private string targetPosePublisherTopic;
    [SerializeField]
    private GameObject targetEEF;
    EndEffectorPos targetEndEffectorPos;


    void Awake()
    {
        targetEndEffectorPos = targetEEF.GetComponent<EndEffectorPos>();
    }
    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(targetPosePublisherTopic);
    }

    // Update is called once per frame
    void Update()
    {
        PoseMsg targetPose = new PoseMsg()
        {
            position = new PointMsg(targetEndEffectorPos.Position.x, targetEndEffectorPos.Position.y, targetEndEffectorPos.Position.z),
            orientation = new QuaternionMsg(targetEndEffectorPos.Orientation.x, targetEndEffectorPos.Orientation.y, targetEndEffectorPos.Orientation.z, targetEndEffectorPos.Orientation.w)
        };
        ros.Publish(targetPosePublisherTopic, targetPose);
    }
}
