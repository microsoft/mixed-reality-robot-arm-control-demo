using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Mrirac;

public class HologramMeshManager : MonoBehaviour
{
    [SerializeField]
    private string hologramObstacleTopic;

    // ROS Connector
    ROSConnection ros;
    float lastSent;

    [SerializeField]
    private float rate;

    [SerializeField]
    private GameObject robot;

    [SerializeField]
    private GameObject hologramObstacles;

    MeshFilter[] hologramObstacleMeshFilters;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<MeshObstaclesMsg>(hologramObstacleTopic);

        lastSent = Time.time;

    }

    // Update is called once per frame
    void Update()
    {

        if (Time.time - lastSent > 1 / rate)
        {
            PublishHologramMeshes();
        }

    }

    void PublishHologramMeshes()
    {
        hologramObstacleMeshFilters = hologramObstacles.GetComponentsInChildren<MeshFilter>();
        MeshObstacleMsg[] hologramObstacleMsgs = new MeshObstacleMsg[hologramObstacleMeshFilters.Length];

        int obstacleIdx = 0;
        foreach (var hologramFilter in hologramObstacleMeshFilters)
        {
            MeshObstacleMsg meshObstacleMsg = new MeshObstacleMsg
            {
                mesh = MriracUtilities.MeshToMeshMsg(hologramFilter.mesh, hologramFilter.gameObject),
                mesh_pose = MriracUtilities.PoseInRobotFrameMsg(robot, hologramFilter.gameObject),
                name = hologramFilter.gameObject.name
            };

            hologramObstacleMsgs[obstacleIdx++] = meshObstacleMsg;

        }

        ros.Publish(hologramObstacleTopic, new MeshObstaclesMsg(hologramObstacleMsgs));
    }

    public void AddHologramObstacle(GameObject obstaclePrefab)
    {
        GameObject hologramObstacle = Instantiate(obstaclePrefab);
        hologramObstacle.name = System.Guid.NewGuid().ToString();
        hologramObstacle.transform.SetParent(hologramObstacles.transform);

        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
        hologramObstacle.transform.position = inFrontOfCameraPos;
    }

    public void ClearHologramObstacles()
    {
        foreach (Transform obstacleTransform in hologramObstacles.transform)
        {
            Destroy(obstacleTransform.gameObject);
        }
    }

}
