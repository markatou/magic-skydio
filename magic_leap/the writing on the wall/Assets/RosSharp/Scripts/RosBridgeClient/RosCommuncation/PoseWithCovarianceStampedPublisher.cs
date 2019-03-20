/*
© Siemens AG, 2017-2018
Author: David Whitney (david_whitney@brown.edu)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class PoseWithCovarianceStampedPublisher : UnityPublisher<Messages.Geometry.PoseWithCovarianceStamped>
    {
        public Transform PublishedTransform;
        public string FrameId = "map";

        private Messages.Geometry.PoseWithCovarianceStamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void InitializeMessage()
        {

            message = new Messages.Geometry.PoseWithCovarianceStamped
            {
                header = new Messages.Standard.Header
                {
                    frame_id = FrameId
                }
            };
            message.pose.covariance = new float[] { 0.25f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.25f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.06853891945200f };
            
        }

        public void UpdateMessage()
        {
            message.header.Update();
            message.pose.pose.position = GetGeometryPoint(PublishedTransform.localPosition.Unity2Ros());
            message.pose.pose.orientation = GetGeometryQuaternion(PublishedTransform.localRotation.Unity2Ros());

            Publish(message);
        }

        private Messages.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            Messages.Geometry.Point geometryPoint = new Messages.Geometry.Point();
            geometryPoint.x = position.x;
            geometryPoint.y = position.y;
            geometryPoint.z = position.z;
            return geometryPoint;
        }

        private Messages.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
        {
            Messages.Geometry.Quaternion geometryQuaternion = new Messages.Geometry.Quaternion();
            geometryQuaternion.x = quaternion.x;
            geometryQuaternion.y = quaternion.y;
            geometryQuaternion.z = quaternion.z;
            geometryQuaternion.w = quaternion.w;
            return geometryQuaternion;
        }

    }
}
