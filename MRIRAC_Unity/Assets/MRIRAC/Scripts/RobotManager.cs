using UnityEngine;

public class RobotManager : MonoBehaviour
{

    [SerializeField]
    private string[] linkTags;
    RobotJoint[] robotJoints;
    GameObject[] joints;

    float[] prevAnglesDeg;
    public float[] PreviousJointAnglesDeg { get => prevAnglesDeg; private set { prevAnglesDeg = value; } }


    void Awake()
    {
        robotJoints = new RobotJoint[linkTags.Length];
        joints = new GameObject[linkTags.Length];
        prevAnglesDeg = new float[linkTags.Length];
        for (int jointIdx = 0; jointIdx < linkTags.Length; jointIdx++)
        {
            joints[jointIdx] = GameObject.FindGameObjectWithTag(linkTags[jointIdx]);
            robotJoints[jointIdx] = joints[jointIdx].GetComponent<RobotJoint>();
        }
    }

    public void SetJointAngles(float[] angles)
    {
        // TrhowIfnotEqual
        for (int jointAngleIdx = 0; jointAngleIdx < angles.Length; jointAngleIdx++)
        {
            joints[jointAngleIdx].transform.localRotation = joints[jointAngleIdx].transform.localRotation * Quaternion.AngleAxis(angles[jointAngleIdx] - prevAnglesDeg[jointAngleIdx], robotJoints[jointAngleIdx].axis);
            prevAnglesDeg[jointAngleIdx] = angles[jointAngleIdx];
        }
    }
}
