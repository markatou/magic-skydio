using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;

public class HandTrackScript : MonoBehaviour
{
    #region  Public Variables
    public enum HandPoses { Ok, Finger, Thumb, OpenHandBack, Fist, NoPose };
    public HandPoses pose = HandPoses.NoPose;
    public Vector3[] pos;
    public GameObject sphereThumb, sphereIndex, sphereWrist;
    public GameObject gestureReceived;
    private string gesture;
    #endregion

    #region Private Variables
    private MLHandKeyPose[] _gestures;
    #endregion


    #region Unity Methods
    private void Awake()
    {
        MLHands.Start();
        _gestures = new MLHandKeyPose[5];
        _gestures[0] = MLHandKeyPose.Ok;
        _gestures[1] = MLHandKeyPose.Finger;
        _gestures[2] = MLHandKeyPose.OpenHandBack;
        _gestures[3] = MLHandKeyPose.Fist;
        _gestures[4] = MLHandKeyPose.Thumb;
        Debug.Log("Oh, magic leap");
        MLHands.KeyPoseManager.EnableKeyPoses(_gestures, true, false);
        Debug.Log("Droneeeees");
        pos = new Vector3[3];
    }
    private void OnDestroy()
    {
        MLHands.Stop();
    }

    private void Update()
    {
        if (GetGesture(MLHands.Left, MLHandKeyPose.Ok))
        {
            pose = HandPoses.Ok;
            gesture = "Okay";
        }
        else if (GetGesture(MLHands.Left, MLHandKeyPose.Finger))
        {
            pose = HandPoses.Finger;
            gesture = "Finger";
        }
        else if (GetGesture(MLHands.Left, MLHandKeyPose.OpenHandBack))
        {
            pose = HandPoses.OpenHandBack;
            gesture = "OpenHand";
        }
        else if (GetGesture(MLHands.Left, MLHandKeyPose.Fist))
        {
            pose = HandPoses.Fist;
            gesture = "Fist";
        }
        else if (GetGesture(MLHands.Left, MLHandKeyPose.Thumb))
        {
            pose = HandPoses.Thumb;
            gesture = "Thumb";
        }
        else
        {
            pose = HandPoses.NoPose;
            gesture = "Nope";
        }

        if (pose != HandPoses.NoPose) ShowPoints();

        gestureReceived = GameObject.Find("Received");
        gestureReceived.GetComponent<TextMesh>().text = gesture;
        Debug.Log("Received" + gesture);
    }
    #endregion

    #region Private Methods
    private void ShowPoints()
    {
        // Left Hand Thumb tip
        pos[0] = MLHands.Left.Thumb.KeyPoints[0].Position;
        // Left Hand Index finger tip 
        pos[1] = MLHands.Left.Index.KeyPoints[0].Position;
        // Left Hand Wrist 
        pos[2] = MLHands.Left.Wrist.KeyPoints[0].Position;
        sphereThumb.transform.position = pos[0];
        sphereIndex.transform.position = pos[1];
        sphereWrist.transform.position = pos[2];
    }

    private bool GetGesture(MLHand hand, MLHandKeyPose type)
    {
        if (hand != null)
        {
            if (hand.KeyPose == type)
            {
                if (hand.KeyPoseConfidence > 0.9f)
                {
                    return true;
                }
            }
        }
        return false;
    }
    #endregion

}