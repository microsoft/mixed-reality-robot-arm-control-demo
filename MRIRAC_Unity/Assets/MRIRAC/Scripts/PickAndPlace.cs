using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Mrirac;
using RosMessageTypes.Trajectory;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using Microsoft.MixedReality.SampleQRCodes;
using System;

public class PickAndPlace : MonoBehaviour
{

    ROSConnection ros;
    [Header("Services and Topics")]
    [SerializeField]
    private string planPickPlaceServiceName;
    [SerializeField]
    private string startPickPlaceServiceName;
    [SerializeField]
    private string pickAndPlaceFeedbackTopic;

    [Header("Pick and Place Targets")]
    [SerializeField]
    private GameObject pick;
    [SerializeField]
    private GameObject place;

    [SerializeField]
    private GameObject pickEEF;
    [SerializeField]
    private GameObject prePickEEF;
    [SerializeField]
    private GameObject placeEEF;
    [SerializeField]
    private GameObject prePlaceEEF;
    EndEffectorPos pickTarget;
    EndEffectorPos placeTarget;
    EndEffectorPos prePickTarget;
    EndEffectorPos prePlaceTarget;

    [Header("")]
    [SerializeField]
    private GameObject qrCodeManager;
    QRCodesVisualizer qrVis;

    [SerializeField]
    private GameObject hologramObstacles;

    [SerializeField]
    [Range(10.0f, 100.0f)]
    private float planVizualizationSpeed;

    TrajectoryPlanner trajectoryPlanner;


    void Awake()
    {
        pickTarget = pickEEF.GetComponent<EndEffectorPos>();
        placeTarget = placeEEF.GetComponent<EndEffectorPos>();
        prePickTarget = prePickEEF.GetComponent<EndEffectorPos>();
        prePlaceTarget = prePlaceEEF.GetComponent<EndEffectorPos>();

        trajectoryPlanner = GetComponent<TrajectoryPlanner>();

        qrVis = qrCodeManager.GetComponent<QRCodesVisualizer>();
    }

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PlanPickAndPlaceRequest, PlanPickAndPlaceResponse>(planPickPlaceServiceName);
        ros.RegisterRosService<StartPickAndPlaceRequest, StartPickAndPlaceResponse>(startPickPlaceServiceName);
        ros.Subscribe<PickAndPlaceActionFeedbackMsg>(pickAndPlaceFeedbackTopic, PickAndPlaceFeedback);
    }

    PoseMsg[] PickAndPlacePoses()
    {
        PoseMsg preGraspPose = new PoseMsg()
        {
            position = new PointMsg(prePickTarget.Position.x, prePickTarget.Position.y, prePickTarget.Position.z),
            orientation = new QuaternionMsg(prePickTarget.Orientation.x, prePickTarget.Orientation.y, prePickTarget.Orientation.z, prePickTarget.Orientation.w)
        };

        PoseMsg graspPose = new PoseMsg()
        {
            position = new PointMsg(pickTarget.Position.x, pickTarget.Position.y, pickTarget.Position.z),
            orientation = new QuaternionMsg(pickTarget.Orientation.x, pickTarget.Orientation.y, pickTarget.Orientation.z, pickTarget.Orientation.w)
        };


        PoseMsg prePlacePose = new PoseMsg()
        {
            position = new PointMsg(prePlaceTarget.Position.x, prePlaceTarget.Position.y, prePlaceTarget.Position.z),
            orientation = new QuaternionMsg(prePlaceTarget.Orientation.x, prePlaceTarget.Orientation.y, prePlaceTarget.Orientation.z, prePlaceTarget.Orientation.w)
        };

        PoseMsg placePose = new PoseMsg()
        {
            position = new PointMsg(placeTarget.Position.x, placeTarget.Position.y, placeTarget.Position.z),
            orientation = new QuaternionMsg(placeTarget.Orientation.x, placeTarget.Orientation.y, placeTarget.Orientation.z, placeTarget.Orientation.w)
        };

        return new PoseMsg[] { preGraspPose, graspPose, prePlacePose, placePose };
    }

    public void CallPlanPickAndPlace()
    {
        PoseMsg[] pickAndPlacePoses = PickAndPlacePoses();

        PlanPickAndPlaceRequest request = new PlanPickAndPlaceRequest(pickAndPlacePoses[0], pickAndPlacePoses[1], pickAndPlacePoses[2], pickAndPlacePoses[3]);

        ros.SendServiceMessage<PlanPickAndPlaceResponse>(planPickPlaceServiceName, request, PlanPickAndPlaceCallback);
    }
    void PlanPickAndPlaceCallback(PlanPickAndPlaceResponse response)
    {
        if (response.success)
        {
            JointTrajectoryPointMsg[] preGraspPoints = response.pre_grasp_trajectory.points;
            JointTrajectoryPointMsg[] graspPoints = response.grasp_trajectory.points;
            JointTrajectoryPointMsg[] prePlacePoints = response.pre_place_trajectory.points;
            JointTrajectoryPointMsg[] placePoints = response.place_trajectory.points;
            JointTrajectoryPointMsg[] combined = preGraspPoints.Concat(graspPoints).ToArray();
            combined = combined.Concat(prePlacePoints).ToArray();
            combined = combined.Concat(placePoints).ToArray();

            trajectoryPlanner.robotUpdater.enabled = false;
            StartCoroutine(trajectoryPlanner.ShowTrajectory(combined, planVizualizationSpeed, endWait: 0.5f));
        }

    }

    public void CallPickAndPlace()
    {
        PoseMsg[] pickAndPlacePoses = PickAndPlacePoses();

        StartPickAndPlaceRequest request = new StartPickAndPlaceRequest(pickAndPlacePoses[0], pickAndPlacePoses[1], pickAndPlacePoses[2], pickAndPlacePoses[3]);

        Debug.Log("calling pick and place");
        ros.SendServiceMessage<StartPickAndPlaceResponse>(startPickPlaceServiceName, request, (o) => Reset());

    }

    void PickAndPlaceFeedback(PickAndPlaceActionFeedbackMsg feedbackMsg)
    {

        Debug.Log("received feedback");
        trajectoryPlanner.robotUpdater.enabled = false;

        JointTrajectoryPointMsg[] trajectoryPoints = feedbackMsg.feedback.trajectory.points;

        StartCoroutine(trajectoryPlanner.ShowTrajectory(trajectoryPoints, vizSpeed: 100.0f, endWait: 0.1f));

        foreach (Transform obstacle in hologramObstacles.transform)
        {
            if (obstacle.gameObject.name == "pickObject")
            {
                Destroy(obstacle.gameObject);
            }
        }
    }

    public void Reset()
    {
        qrVis.pickActive = false;

        foreach (Transform obstacle in hologramObstacles.transform)
        {
            if (obstacle.gameObject.name == "pickObject")
            {
                Destroy(obstacle.gameObject);
            }
        }


    }

    public void Setup()
    {
        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 1.0f) + Camera.main.transform.position;
        pick.transform.position = inFrontOfCameraPos + Camera.main.transform.right * 0.1f;
        place.transform.position = inFrontOfCameraPos - Camera.main.transform.right * 0.1f;
    }
}
