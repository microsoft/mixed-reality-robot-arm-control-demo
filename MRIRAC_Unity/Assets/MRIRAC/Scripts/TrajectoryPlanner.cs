using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Mrirac;
using RosMessageTypes.Geometry;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Std;
using System;
using System.Linq;

public class TrajectoryPlanner : MonoBehaviour
{
    private ROSConnection ros;

    [SerializeField]
    private string plannerServiceName;
    [SerializeField]
    private string executorServiceName;

    [SerializeField]
    private GameObject robot;
    [SerializeField]
    private GameObject root;
    [SerializeField]
    private GameObject targetObject;
    [SerializeField]
    private GameObject targetEndEffector;
    SetTargetShaders setTargetShaders;

    EndEffectorPos planningTarget;

    public UpdateRobot robotUpdater;
    RobotManager robotManager;



    // Trajectory Lines
    bool showLines;
    [Header("Visualization Options")]
    [SerializeField]
    private GameObject linePrefab;
    [SerializeField]
    [Range(10.0f, 100.0f)]
    private float planVizualizationSpeed;
    static string[] linkLineTags = { "end_effector", "link_2", "link_3", "link_4", "link_5", "link_6" };
    GameObject[] joints = new GameObject[linkLineTags.Length];
    GameObject[] lines = new GameObject[linkLineTags.Length];
    LineRenderer[] lineRenderers = new LineRenderer[linkLineTags.Length];

    float[] lastTrajAngles;


    void Awake()
    {
        planningTarget = targetEndEffector.GetComponent<EndEffectorPos>();
        robotUpdater = robot.GetComponent<UpdateRobot>();
        robotManager = robot.GetComponent<RobotManager>();
        setTargetShaders = targetObject.GetComponent<SetTargetShaders>();
    }

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<TrajectoryPlanRequest, TrajectoryPlanResponse>(plannerServiceName);
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(executorServiceName);


        for (int i = 0; i < linkLineTags.Length; i++)
        {
            lines[i] = Instantiate(linePrefab);
            lineRenderers[i] = lines[i].GetComponent<LineRenderer>();
            joints[i] = GameObject.FindGameObjectWithTag(linkLineTags[i]);
        }

    }

    public void CallTrajectoryPlanner()
    {
        PoseMsg targetPose = new PoseMsg()
        {
            position = new PointMsg(planningTarget.Position.x, planningTarget.Position.y, planningTarget.Position.z),
            orientation = new QuaternionMsg(planningTarget.Orientation.x, planningTarget.Orientation.y, planningTarget.Orientation.z, planningTarget.Orientation.w)
        };

        TrajectoryPlanRequest request = new TrajectoryPlanRequest(targetPose);

        ros.SendServiceMessage<TrajectoryPlanResponse>(plannerServiceName, request, TrajectoryPlannerCallback);
    }


    void TrajectoryPlannerCallback(TrajectoryPlanResponse response)
    {
        if (response.success)
        {
            robotUpdater.enabled = false;
            JointTrajectoryMsg trajectory = response.trajectory;

            JointTrajectoryPointMsg[] points = trajectory.points;

            StartCoroutine(ShowTrajectory(points, planVizualizationSpeed, endWait: 3.0f));

        }
        else
        {
            Debug.Log("No motion plan found!");
            StartCoroutine(setTargetShaders.ShowError());
        }
    }

    public IEnumerator ShowTrajectory(JointTrajectoryPointMsg[] points, float vizSpeed, float endWait)
    {
        //float[] prevJointAnglesDeg = robotManager.prevAnglesDeg;
        float[] prevJointAnglesDeg = robotManager.PreviousJointAnglesDeg;

        for (int pointIdx = 0; pointIdx < points.Length; pointIdx++)
        {

            JointTrajectoryPointMsg point = points[pointIdx];

            float[] jointAnglesDeg = point.positions.Select(angleRad => (float)angleRad * Mathf.Rad2Deg).ToArray();

            if (pointIdx == points.Length - 1)
            {
                lastTrajAngles = jointAnglesDeg;
            }

            // bool to check whether interpolation causes the angle to change more than 180deg, which results in strange visuals
            bool spin = false;

            int numOfInterpSteps = 5;
            for (int iter = 0; iter < numOfInterpSteps; iter++)
            {
                float[] interpolatedJointAngles = new float[point.positions.Length];
                for (int angleIdx = 0; angleIdx < interpolatedJointAngles.Length; angleIdx++)
                {
                    if (Mathf.Abs(prevJointAnglesDeg[angleIdx] - jointAnglesDeg[angleIdx]) > 180f)
                    {
                        spin = true;
                    }
                    interpolatedJointAngles[angleIdx] = Mathf.Lerp(prevJointAnglesDeg[angleIdx], jointAnglesDeg[angleIdx], iter * 0.2f);
                }
                if (spin)
                {
                    break;
                }
                robotManager.SetJointAngles(interpolatedJointAngles);
                yield return new WaitForSeconds(1 / vizSpeed);
            }

            if (showLines)
            {
                for (int lineRendererIdx = 0; lineRendererIdx < lineRenderers.Length; lineRendererIdx++)
                {
                    lineRenderers[lineRendererIdx].positionCount = lineRenderers[lineRendererIdx].positionCount + 1;
                    lineRenderers[lineRendererIdx].SetPosition(pointIdx, joints[lineRendererIdx].transform.position);
                }
            }
            robotManager.SetJointAngles(jointAnglesDeg);
            yield return new WaitForSeconds(1 / vizSpeed);

            prevJointAnglesDeg = jointAnglesDeg;

        }

        yield return new WaitForSeconds(endWait);

        for (int j = 0; j < lineRenderers.Length; j++)
        {
            lineRenderers[j].positionCount = 0;
        }
        robotUpdater.enabled = true;

    }

    public void CallTrajectoryExecutor()
    {
        ros.SendServiceMessage<EmptyResponse>(executorServiceName, new EmptyRequest(), (o) => Debug.Log("executed planned trajectory"));
    }

    public void ToggleLines() { showLines = !showLines; }
}