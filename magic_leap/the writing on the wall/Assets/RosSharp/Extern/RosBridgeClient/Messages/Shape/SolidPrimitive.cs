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

namespace RosSharp.RosBridgeClient.Messages.Shape
{
    public class SolidPrimitive : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "shape_msgs/SolidPrimitive";
        public byte BOX = 1;
        public byte SPHERE = 2;
        public byte CYLINDER = 3;
        public byte CONE = 4;
        public byte BOX_X = 0;
        public byte BOX_Y = 1;
        public byte BOX_Z = 2;
        public byte SPHERE_RADIUS = 0;
        public byte CYLINDER_HEIGHT = 0;
        public byte CYLINDER_RADIUS = 1;
        public byte CONE_HEIGHT = 0;
        public byte CONE_RADIUS = 1;
        public byte type;
        public float[] dimensions;
        public SolidPrimitive()
        {
            type = 0;
            dimensions = new float[0];
        }
    }

}