using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class ClearObstacles : MonoBehaviour
{
    ROSConnection ros;

    [SerializeField]
    private string clearObstaclesServiceName;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<EmptyRequest, EmptyResponse>(clearObstaclesServiceName);

    }

    public void CallClearObstacles()
    {
        ros.SendServiceMessage<EmptyResponse>(clearObstaclesServiceName, new EmptyRequest());
    }
}
