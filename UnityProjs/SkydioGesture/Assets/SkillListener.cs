using UnityEngine;
using System;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;

namespace RosSharp.RosBridgeClient
{
    public class SkillListener : UnitySubscriber<Messages.Standard.String>
    {
        public RosSocket rosSocket;
        public GameObject skillSuccess;
        public string subscribedString;

        protected override void Start()
        {
            //rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol("ws://192.168.1.231:9090"));

            Debug.Log("confirm that string start works");
            base.Start();
            skillSuccess.GetComponent<TextMesh>().text = "Waiting for skill activation.";

        }

        void Update()
        {
            if (subscribedString == "skill set")
            {
                Debug.Log("Success");
                skillSuccess.GetComponent<TextMesh>().text = "Skill Activated!";

            }
        }

        protected override void ReceiveMessage(Messages.Standard.String message)
        {
            Debug.Log(message.data);
            subscribedString = message.data;
        }

        void SubscriptionHandlerStatus(std_msgs.String message)
        {
            Debug.Log("Status: " + (message).data);
            subscribedString = message.data;
        }
    }
}