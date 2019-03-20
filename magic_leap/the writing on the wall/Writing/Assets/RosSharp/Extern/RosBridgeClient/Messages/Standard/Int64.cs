using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.Messages.Standard
{
    public class Int64 : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "std_msgs/Int64";
        public int data;

        public Int64()
        {
            data = 0;
        }

        public Int64(int data)
        {
            this.data = data;
        }
    }
}