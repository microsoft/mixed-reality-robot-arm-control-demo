using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class HomeArm : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string homeServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(homeServiceName);

    }

    public void CallHomeArm()
    {
        ros.SendServiceMessage<EmptyResponse>(homeServiceName, new EmptyRequest());
    }
}
