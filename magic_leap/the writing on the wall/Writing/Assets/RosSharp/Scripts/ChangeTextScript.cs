using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.Messages.Standard;
using geom_msgs = RosSharp.RosBridgeClient.Messages.Geometry;
using UnityEngine;
using System;

public class ChangeTextScript : MonoBehaviour
{


    void Start()
    {
        TextMesh textObject = GameObject.Find("text").GetComponent<TextMesh>();
        textObject.text = "test";


    }

}