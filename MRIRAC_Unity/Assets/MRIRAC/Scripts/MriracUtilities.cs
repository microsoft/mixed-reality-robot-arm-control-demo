using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Shape;
using System.Linq;

public static class MriracUtilities
{
    public static PoseMsg PoseInRobotFrameMsg(GameObject robot, GameObject gameObject)
    {
        Vector3<FLU> positionInRobotFrame;

        Quaternion<FLU> orientationInRobotFrame;

        Matrix4x4 worldToRobotTransformMat;

        worldToRobotTransformMat = robot.transform.worldToLocalMatrix;
        positionInRobotFrame = (worldToRobotTransformMat.MultiplyVector(gameObject.transform.position - robot.transform.position)).To<FLU>();
        orientationInRobotFrame = (worldToRobotTransformMat.rotation * Quaternion.Euler(gameObject.transform.eulerAngles.x, gameObject.transform.eulerAngles.y, gameObject.transform.eulerAngles.z)).To<FLU>();

        PoseMsg poseMsg = new PoseMsg()
        {
            position = new PointMsg(positionInRobotFrame.x, positionInRobotFrame.y, positionInRobotFrame.z),
            orientation = new QuaternionMsg(orientationInRobotFrame.x, orientationInRobotFrame.y, orientationInRobotFrame.z, orientationInRobotFrame.w)
        };

        return poseMsg;
    }

    public static MeshMsg MeshToMeshMsg(Mesh meshObject, GameObject gameObject)
    {
        Vector3[] vertices = meshObject.vertices;
        uint[] triangles = meshObject.triangles.Select(triangle => (uint)triangle).ToArray();

        PointMsg[] vertexPoints = new PointMsg[vertices.Length];

        for (int vertexIdx = 0; vertexIdx < vertexPoints.Length; vertexIdx++)
        {
            Vector3 vertexPoint = new Vector3(vertices[vertexIdx].x * gameObject.transform.localScale.x, vertices[vertexIdx].y * gameObject.transform.localScale.y, vertices[vertexIdx].z * gameObject.transform.localScale.z);
            Vector3<FLU> fluVertexPoint = vertexPoint.To<FLU>();
            vertexPoints[vertexIdx] = new PointMsg(fluVertexPoint.x, fluVertexPoint.y, fluVertexPoint.z);
        }

        MeshTriangleMsg[] meshTriangleMsgs = new MeshTriangleMsg[triangles.Length / 3];

        int trianglePointIter = 0;
        for (int meshTriangleIdx = 0; meshTriangleIdx < meshTriangleMsgs.Length; meshTriangleIdx++)
        {
            // flipping order of trinagle points so that normals are correct in robot frame
            uint[] triangle = new uint[3];
            triangle[2] = triangles[trianglePointIter];
            triangle[1] = triangles[trianglePointIter + 1];
            triangle[0] = triangles[trianglePointIter + 2];
            meshTriangleMsgs[meshTriangleIdx] = new MeshTriangleMsg(triangle);

            trianglePointIter += 3;
        }

        return new MeshMsg(meshTriangleMsgs, vertexPoints);
    }

    public static MeshMsg MeshToMeshMsgWithRobot(Mesh meshObject, GameObject gameObject, BoxCollider[] robotColliders)
    {

        Vector3[] vertices = meshObject.vertices;
        int[] triangles = meshObject.triangles;

        List<uint> validTrangles = new List<uint>(triangles.Length);

        List<BoxCollider> potentialCollisions = new List<BoxCollider>();

        foreach (var collider in robotColliders)
        {
            if (meshObject.bounds.Intersects(collider.bounds))
            {
                potentialCollisions.Add(collider);
            }
        }

        if (potentialCollisions.Count == 0)
        {
            validTrangles = triangles.Select(triangle => (uint)triangle).ToList();
        }
        else
        {
            for (int triangleIdx = 0; triangleIdx < triangles.Length - 3; triangleIdx += 3)
            {
                bool invalid = false;
                foreach (var collider in potentialCollisions)
                {
                    if (PointInOABB(vertices[triangles[triangleIdx]], collider) || PointInOABB(vertices[triangles[triangleIdx + 1]], collider) || PointInOABB(vertices[triangles[triangleIdx + 2]], collider))
                    {
                        invalid = true;
                        break;
                    }
                }

                if (invalid)
                {
                    continue;
                }
                else
                {
                    validTrangles.Add((uint)triangles[triangleIdx]);
                    validTrangles.Add((uint)triangles[triangleIdx + 1]);
                    validTrangles.Add((uint)triangles[triangleIdx + 2]);
                }
            }
        }

        PointMsg[] vertexPoints = new PointMsg[vertices.Length];

        for (uint vertexPointIdx = 0; vertexPointIdx < vertexPoints.Length; vertexPointIdx++)
        {
            Vector3 vertexPoint = new Vector3(vertices[vertexPointIdx].x * gameObject.transform.localScale.x, vertices[vertexPointIdx].y * gameObject.transform.localScale.y, vertices[vertexPointIdx].z * gameObject.transform.localScale.z);
            Vector3<FLU> fluVertexPoint = vertexPoint.To<FLU>();
            vertexPoints[vertexPointIdx] = new PointMsg(fluVertexPoint.x, fluVertexPoint.y, fluVertexPoint.z);
        }

        MeshTriangleMsg[] meshTriangleMsgs = new MeshTriangleMsg[validTrangles.Count / 3];

        int trianglePointIter = 0;
        for (int meshTriangleIdx = 0; meshTriangleIdx < meshTriangleMsgs.Length; meshTriangleIdx++)
        {
            // flipping order of trinagle points so that normals are correct in robot frame
            uint[] triangle = new uint[3];
            triangle[2] = validTrangles[trianglePointIter];
            triangle[1] = validTrangles[trianglePointIter + 1];
            triangle[0] = validTrangles[trianglePointIter + 2];
            meshTriangleMsgs[meshTriangleIdx] = new MeshTriangleMsg(triangle);

            trianglePointIter += 3;
        }

        return new MeshMsg(meshTriangleMsgs, vertexPoints);
    }

    public static bool PointInOABB(Vector3 point, BoxCollider box)
    {
        point = box.transform.InverseTransformPoint(point);
        Vector3 center = box.center;
        Vector3 size = box.size;
        float x = size.x * 0.5f + point.x - center.x;
        if (x < 0 || x > size.x)
            return false;
        float y = size.y * 0.5f + point.y - center.y;
        if (y < 0 || y > size.y)
            return false;
        float z = size.z * 0.5f + point.z - center.z;
        if (z < 0 || z > size.z)
            return false;
        return true;
    }
}
