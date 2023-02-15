using UnityEngine;
using Microsoft.MixedReality.Toolkit.SpatialAwareness;
using Microsoft.MixedReality.Toolkit;
using RosMessageTypes.Shape;
using RosMessageTypes.Mrirac;
using Unity.Robotics.ROSTCPConnector;
using Microsoft.MixedReality.Toolkit.XRSDK.OpenXR;

public class SpatialMeshManager : MonoBehaviour, IMixedRealitySpatialAwarenessObservationHandler<SpatialAwarenessMeshObject>
{
    [SerializeField]
    private string spatialMeshObstacleTopic;

    // ROS Connector
    ROSConnection ros;

    [SerializeField]
    private GameObject robot;
    BoxCollider[] boxColliders;
    bool enablePublishing = false;
    bool meshToggle = false;


    void Awake()
    {
        boxColliders = robot.GetComponentsInChildren<BoxCollider>();

    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<MeshObstacleMsg>(spatialMeshObstacleTopic);
    }

    private void OnEnable()
    {
        // Register component to listen for Mesh Observation events, typically done in OnEnable()
        CoreServices.SpatialAwarenessSystem.RegisterHandler<IMixedRealitySpatialAwarenessObservationHandler<SpatialAwarenessMeshObject>>(this);
        HideMesh();
    }

    private void OnDisable()
    {
        // Unregister component from Mesh Observation events, typically done in OnDisable()
        CoreServices.SpatialAwarenessSystem.UnregisterHandler<IMixedRealitySpatialAwarenessObservationHandler<SpatialAwarenessMeshObject>>(this);
    }

    public virtual void OnObservationAdded(MixedRealitySpatialAwarenessEventData<SpatialAwarenessMeshObject> eventData)
    {
        if (enablePublishing)
        {
            MeshMsg meshMsg = MriracUtilities.MeshToMeshMsgWithRobot(
                meshObject: eventData.SpatialObject.Filter.mesh,
                gameObject: eventData.SpatialObject.Filter.gameObject,
                robotColliders: boxColliders);

            MeshObstacleMsg meshObstacleMsg = new MeshObstacleMsg
            {
                mesh = meshMsg,
                mesh_pose = MriracUtilities.PoseInRobotFrameMsg(robot, eventData.SpatialObject.Filter.gameObject),
                name = eventData.Id.ToString()
            };

            Debug.Log("Publishing added spatial mesh");
            ros.Publish(spatialMeshObstacleTopic, meshObstacleMsg);
        }

    }

    public virtual void OnObservationUpdated(MixedRealitySpatialAwarenessEventData<SpatialAwarenessMeshObject> eventData)
    {
        if (enablePublishing)
        {
            MeshMsg meshMsg = MriracUtilities.MeshToMeshMsgWithRobot(
                meshObject: eventData.SpatialObject.Filter.mesh,
                gameObject: eventData.SpatialObject.Filter.gameObject,
                robotColliders: boxColliders);

            MeshObstacleMsg meshObstacleMsg = new MeshObstacleMsg
            {
                mesh = meshMsg,
                mesh_pose = MriracUtilities.PoseInRobotFrameMsg(robot, eventData.SpatialObject.Filter.gameObject),
                name = eventData.Id.ToString()
            };

            Debug.Log("Publishing updated spatial mesh");
            ros.Publish(spatialMeshObstacleTopic, meshObstacleMsg);
        }

    }

    public virtual void OnObservationRemoved(MixedRealitySpatialAwarenessEventData<SpatialAwarenessMeshObject> eventData)
    {
        // Do stuff
        MeshObstacleMsg meshObstacleMsg = new MeshObstacleMsg
        {
            action = "remove",
            name = eventData.Id.ToString()
        };

        Debug.Log("Publishing message to remove spatial mesh");
        ros.Publish(spatialMeshObstacleTopic, meshObstacleMsg);

    }

    public void StartMeshObservers()
    {
        // Resume Mesh Observation from all Observers
        CoreServices.SpatialAwarenessSystem.ResumeObservers();
    }

    public void StoptMeshObservers()
    {
        // Resume Mesh Observation from all Observers
        CoreServices.SpatialAwarenessSystem.SuspendObservers();
    }

    public void ShowMesh()
    {
        // Get the first Mesh Observer available, generally we have only one registered
        var observer = CoreServices.GetSpatialAwarenessSystemDataProvider<OpenXRSpatialAwarenessMeshObserver>();

        // Set to visible and the Occlusion material
        observer.DisplayOption = SpatialAwarenessMeshDisplayOptions.Visible;
    }

    public void HideMesh()
    {
        // Get the first Mesh Observer available, generally we have only one registered
        var observer = CoreServices.GetSpatialAwarenessSystemDataProvider<OpenXRSpatialAwarenessMeshObserver>();

        // Set to not visible
        observer.DisplayOption = SpatialAwarenessMeshDisplayOptions.None;
        Debug.Log(observer.Name);
    }

    public void TogglePublishing()
    {
        enablePublishing = !enablePublishing;
    }

    public void MeshToggle()
    {
        meshToggle = !meshToggle;

        if (meshToggle)
        {
            ShowMesh();
        }
        else
        {
            HideMesh();
        }
    }
}
