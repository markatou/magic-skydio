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
    public class PointArraySubscriber : UnitySubscriber<Messages.Fetch.PointArray>
    {
        public Vector3[] SubscribedArray;

        protected override void Start()
        {
            base.Start();
        }

        protected override void ReceiveMessage(Messages.Fetch.PointArray message)
        {   
            SubscribedArray = new Vector3[message.data.Length];
            for(int i = 0; i < message.data.Length; i++)
            {
                SubscribedArray[i] = ToVector3(message.data[i]).Ros2Unity();
            }
        }

        protected Vector3 ToVector3(Messages.Geometry.Point p)
        {
            return new Vector3(p.x, p.y, p.z);
        }
    }
}