using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointSubscriber : MonoBehaviour
{
    public float[] jointStatesDeg;

    [SerializeField]
    private int numberOfJoints;

    [SerializeField]
    private string jointTopic;


    void Awake()
    {
        jointStatesDeg = new float[numberOfJoints];
    }

    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(jointTopic, Updatejoints);

    }

    void Updatejoints(JointStateMsg jointState)
    {
        for (int i = 0; i < numberOfJoints; i++)
        {
            jointStatesDeg[i] = (float)(jointState.position[i] * Mathf.Rad2Deg);
        }

    }
}
