//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Mrirac
{
    [Serializable]
    public class TrajectoryPlanRequest : Message
    {
        public const string k_RosMessageName = "mrirac_msgs/TrajectoryPlan";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PoseMsg target_pose;

        public TrajectoryPlanRequest()
        {
            this.target_pose = new Geometry.PoseMsg();
        }

        public TrajectoryPlanRequest(Geometry.PoseMsg target_pose)
        {
            this.target_pose = target_pose;
        }

        public static TrajectoryPlanRequest Deserialize(MessageDeserializer deserializer) => new TrajectoryPlanRequest(deserializer);

        private TrajectoryPlanRequest(MessageDeserializer deserializer)
        {
            this.target_pose = Geometry.PoseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.target_pose);
        }

        public override string ToString()
        {
            return "TrajectoryPlanRequest: " +
            "\ntarget_pose: " + target_pose.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
