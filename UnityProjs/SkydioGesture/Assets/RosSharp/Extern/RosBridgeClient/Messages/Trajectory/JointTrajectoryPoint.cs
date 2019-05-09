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

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Trajectory
{
    public class JointTrajectoryPoint : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "trajectory_msgs/JointTrajectoryPoint";
        public float[] positions;
        public float[] velocities;
        public float[] accelerations;
        public float[] effort;
        public Standard.Time time_from_start;
        public JointTrajectoryPoint()
        {
            positions = new float[0];
            velocities = new float[0];
            accelerations = new float[0];
            effort = new float[0];
            time_from_start = new Standard.Time();
        }
    }

}