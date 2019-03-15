using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Standard
{
    public class Float32 : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "std_msgs/Float32";
        public float data;

        public Float32()
        {
            data = 0;
        }
    }
}