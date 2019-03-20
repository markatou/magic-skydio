/*
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
    public class PointStampedPublisher : UnityPublisher<Messages.Geometry.PointStamped>
    {
        public Transform PublishedTransform;
        public string FrameId = "map";

        private Messages.Geometry.PointStamped message;

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }

        private void Update()
        {
            UpdateMessage();
        }

        private void InitializeMessage()
        {

            message = new Messages.Geometry.PointStamped()
            {
                header = new Messages.Standard.Header()
                {
                    frame_id = FrameId
                }
            };
        }

        public void UpdateMessage()
        {
            //message.header.stamp = null; if you want to use unix time do this
            message.header.Update();
            message.point = GetGeometryPoint(PublishedTransform.localPosition.Unity2Ros());
            Publish(message);
        }

        private Messages.Geometry.Point GetGeometryPoint(Vector3 position)
        {
            Messages.Geometry.Point geometryPoint = new Messages.Geometry.Point
            {
                x = position.x,
                y = position.y,
                z = position.z
            };
            return geometryPoint;
        }
    }
}
