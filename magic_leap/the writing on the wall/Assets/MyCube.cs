using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;
using UnityEngine.UI;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using geom_msgs = RosSharp.RosBridgeClient.Messages.Geometry;



public class MyCube : MonoBehaviour
{
    public RosSocket rosSocket;
    public string publication_id;
    public MLPersistentBehavior persistentBehavior;
    private MLInputController _controller;
    private const float _distance = 2.0f;
    public String position = "";
    public String orientation = "";
    public String speed = "";

    public Vector3 drone_pos;
    public Vector3 temp_pos;
    public GameObject drone_follower;
    public bool start = false;


    public TextMesh textObject;
    public TextMesh orientObject;
    public TextMesh speedObject;

    private void Start()
    {
        // WebSocket Connection:
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol("ws://192.168.1.231:9090"));


        // Publication:
        publication_id = rosSocket.Advertise<std_msgs.Int64>("go_forward");

        // Subscription:
        string subscription_id1 = rosSocket.Subscribe<geom_msgs.Pose>("/pose", SubscriptionHandlerPose);
        string subscription_id2 = rosSocket.Subscribe<std_msgs.Float32>("/speed", SubscriptionHandlerSpeed);

        drone_pos = new Vector3((float)0.4, (float)-0.7, 4);
        temp_pos = new Vector3(0, 0, 0);
    }

    private void Awake()
    {
        textObject = GameObject.Find("text").GetComponent<TextMesh>();
        orientObject = GameObject.Find("orientation").GetComponent<TextMesh>();
        speedObject = GameObject.Find("speed").GetComponent<TextMesh>();
        drone_follower = GameObject.Find("drone_follower");

        MLInput.Start();
        _controller = MLInput.GetController(MLInput.Hand.Left);


    }
    private void OnDestroy()
    {
        MLInput.Stop();
    }
    void Update()
    {
        if (_controller.TriggerValue > 0.2f)
        {
            Debug.Log("Sending");
            std_msgs.Int64 message = new std_msgs.Int64(1);
            rosSocket.Publish(publication_id, message);
            System.Threading.Thread.Sleep(1000);
        }

        textObject.text = position;
        orientObject.text = orientation;
        speedObject.text = speed;

        Debug.Log("In update: " + drone_pos);
        drone_follower.transform.position = drone_pos;
        temp_pos = drone_follower.transform.position;

    }


    void SubscriptionHandlerPose(geom_msgs.Pose message)
    {
       
        position = "(" + message.position.x+ ", " + message.position.y + ", " + message.position.z + ")";
        orientation = "(" + message.orientation.w + ", " + message.orientation.x + ", " + message.orientation.y +", " + message.orientation.z + ")";
        Debug.Log("Position: " + position);
        drone_pos = new Vector3(temp_pos.x, message.position.z - (float)0.7, temp_pos.z);
        Debug.Log("Position Cube: " + drone_pos);
        // Z 0.8 -> Y 0.1

        start = true;
    }

    void SubscriptionHandlerSpeed(std_msgs.Float32 message)
    {
        Debug.Log("Speed: " + (message).data.ToString("n2"));
        speed = (message).data.ToString();
    }

}