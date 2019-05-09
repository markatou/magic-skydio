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

namespace RosSharp.RosBridgeClient.Messages.MoveIt
{
    public class CollisionObject : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "moveit_msgs/CollisionObject";
        public Standard.Header header;
        public string id;
        public ObjectRecognition.ObjectType type;
        public Shape.SolidPrimitive[] primitives;
        public Geometry.Pose[] primitive_poses;
        public Shape.Mesh[] meshes;
        public Geometry.Pose[] mesh_poses;
        public Shape.Plane[] planes;
        public Geometry.Pose[] plane_poses;
        public byte operation;
        public byte ADD = 0;
        public byte REMOVE = 1;
        public byte APPEND = 2;
        public byte MOVE = 3;
        public CollisionObject()
        {
            header = new Standard.Header();
            id = "";
            type = new ObjectRecognition.ObjectType();
            primitives = new Shape.SolidPrimitive[0];
            primitive_poses = new Geometry.Pose[0];
            meshes = new Shape.Mesh[0];
            mesh_poses = new Geometry.Pose[0];
            planes = new Shape.Plane[0];
            plane_poses = new Geometry.Pose[0];
            operation = 0;
        }
    }

}