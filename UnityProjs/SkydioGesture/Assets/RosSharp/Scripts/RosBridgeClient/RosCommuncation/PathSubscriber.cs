using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PathSubscriber : UnitySubscriber<Messages.Navigation.Path>
    {

        public LineRenderer lineRenderer;
        private bool isMessageReceived;
        private Vector3[] poses;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        private void ProcessMessage()
        {
            lineRenderer.positionCount = poses.Length;
            lineRenderer.SetPositions(poses);
            isMessageReceived = false;
        }

        protected override void ReceiveMessage(Messages.Navigation.Path message)
        {
            poses = (from pose in message.poses select GetPose(pose).Ros2Unity()).ToArray();
            isMessageReceived = true;
        }

        private Vector3 GetPose(Messages.Geometry.PoseStamped message)
        {
            return new Vector3(
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z);
        }

    }
}