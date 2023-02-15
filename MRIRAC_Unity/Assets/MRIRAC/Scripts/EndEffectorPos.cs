using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class EndEffectorPos : MonoBehaviour
{
    [SerializeField]
    private GameObject root;

    Vector3<FLU> position;
    public Vector3<FLU> Position { get => position; private set { position = value; } }

    Quaternion<FLU> orientation;
    public Quaternion<FLU> Orientation { get => orientation; private set { orientation = value; } }

    Matrix4x4 worldToRobotTransformMat;

    // Start is called before the first frame update
    void Start()
    {
        SetPose();

    }

    // Update is called once per frame
    void Update()
    {
        SetPose();

    }

    void SetPose()
    {
        worldToRobotTransformMat = root.transform.worldToLocalMatrix;
        position = (worldToRobotTransformMat.MultiplyVector(transform.position - root.transform.position)).To<FLU>();
        orientation = (worldToRobotTransformMat.rotation * Quaternion.Euler(transform.eulerAngles.x, transform.eulerAngles.y, transform.eulerAngles.z)).To<FLU>();
    }

}
