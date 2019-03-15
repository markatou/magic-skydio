using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using geom_msgs = RosSharp.RosBridgeClient.Messages.Geometry;
using UnityEngine;

public class ChangeTextScript : MonoBehaviour
{
    void Start()
    {
        TextMesh textObject = GameObject.Find("text").GetComponent<TextMesh>();
        textObject.text = "test";

        // WebSocket Connection:
        RosSocket rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol("ws://192.168.1.231:9090"));


        // Publication:
        string publication_id = rosSocket.Advertise<std_msgs.String>("publication_test");
        std_msgs.String message = new std_msgs.String("hello from the magic leap");
        rosSocket.Publish(publication_id, message);

        // Subscription:
        string subscription_id1 = rosSocket.Subscribe<geom_msgs.Pose>("/pose", SubscriptionHandlerPose);
        string subscription_id2 = rosSocket.Subscribe<std_msgs.Float32>("/speed", SubscriptionHandlerSpeed);



    }

    private static void SubscriptionHandlerPose(geom_msgs.Pose message)
    {
        Debug.Log("Position: (" + (message).position.x + ", " + (message).position.y + ", " + (message).position.z+ ")");
        Debug.Log("Orientation: (" + (message).orientation.w + ", " + (message).orientation.x + ", " + (message).orientation.y + ", " + (message).orientation.z + ")");
    }

    private static void SubscriptionHandlerSpeed(std_msgs.Float32 message)
    {
        Debug.Log("Speed: " + (message).data);
    }

}