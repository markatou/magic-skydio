using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class GesturePublisher : UnityPublisher<Messages.Standard.String>
    {
        public GameObject gestureReceived;
        public string FrameId = "base";

        private Messages.Standard.String message;

        protected override void Start()
        {
            base.Start();
            gestureReceived = GameObject.Find("Received");
        }

        private void Update()
        {
            UpdateMessage();
        }

        public void UpdateMessage()
        {
            message = GetGesture(gestureReceived.GetComponent<TextMesh>().text);
            Publish(message);
        }

        private Messages.Standard.String GetGesture(string received)
        {
            Messages.Standard.String gesture = new Messages.Standard.String(received);
            return gesture;
        }
    }
}
