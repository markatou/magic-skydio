using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PoseSubscriberSKYDIO : UnitySubscriber<Messages.Geometry.Pose>
    {

        public Transform PublishedTransform;

        private Vector3 position;
        private Vector3 init_position; //offset after takeoff
        private bool init_position_set = false;
        private Quaternion rotation;
        private bool isMessageReceived;

        protected override void Start()
        {
            base.Start();
        }

        private void Update()
        {
            if (isMessageReceived)
                ProcessMessage();
        }

        protected override void ReceiveMessage(Messages.Geometry.Pose message)
        {
            if (!init_position_set)
            {
                init_position = GetPosition(message).Ros2Unity();
                init_position_set = true;
                Debug.Log("init_position:");
                Debug.Log(init_position.ToString("F8"));
            }
            position = GetPosition(message).Ros2Unity() - init_position;
            rotation = GetRotation(message).Ros2Unity(); // defined in TransformExtensions.cs
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            PublishedTransform.localPosition = position;
            PublishedTransform.localRotation = rotation;
            isMessageReceived = false;
        }

        private Vector3 GetPosition(Messages.Geometry.Pose message)
        {
            Vector3 skydio_position = new Vector3(message.position.x, message.position.y, message.position.z);
            return skydio_position;
        }

        private Quaternion GetRotation(Messages.Geometry.Pose message)
        {
            return new Quaternion(
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w);
        }
    }
}