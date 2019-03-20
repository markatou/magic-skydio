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
    private GameObject _cube, _camera;
    private MLInputController _controller;
    private const float _distance = 2.0f;
    public String position = "";
    public String orientation = "";
    public String speed = "";


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
    }

    private void Awake()
    {
        textObject = GameObject.Find("text").GetComponent<TextMesh>();
        orientObject = GameObject.Find("orientation").GetComponent<TextMesh>();
        speedObject = GameObject.Find("speed").GetComponent<TextMesh>();

        _cube = GameObject.Find("Cube");
        _camera = GameObject.Find("Main Camera");

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

    }


    void SubscriptionHandlerPose(geom_msgs.Pose message)
    {
       
        position = "(" + message.position.x.ToString("n2")+ ", " + message.position.y.ToString("n2") + ", " + message.position.z.ToString("n2") + ")";
        orientation = "(" + message.orientation.w.ToString("n2") + ", " + message.orientation.x.ToString("n2") + ", " + message.orientation.y.ToString("n2") +", " + message.orientation.z.ToString("n2") + ")";
        Debug.Log("Position: " + position);
    }

    void SubscriptionHandlerSpeed(std_msgs.Float32 message)
    {
        Debug.Log("Speed: " + (message).data.ToString("n2"));
        speed = (message).data.ToString("n2");
    }

}