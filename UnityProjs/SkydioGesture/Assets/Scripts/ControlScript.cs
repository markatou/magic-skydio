using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;
using RosSharp.RosBridgeClient;

public class ControlScript : MonoBehaviour
{
    public GameObject converter;
    public GameObject target;
    public GameObject pointer;
    public AudioSource AudioSource;
    public PoseStampedPublisher goalPublisher;
    public PoseWithCovarianceStampedPublisher locationEstimatePublisher;

    private GameObject focus;
    private MLInputController _controller;
    private const float _rotationSpeed = 25.0f;
    private const float _distance = 2.0f;
    private const float _moveSpeed = 0.6f;
    private bool _bumper = false;
    private MLHandKeyPose[] gestures;

    public float DebounceDuration = 2.0f;
    private float TimeLastPublished = 0f;    

    void Awake()
    {
        // start control and hand tracking
        MLInput.Start();
        MLHands.Start();

        // control setup
        MLInput.OnControllerButtonDown += OnButtonDown;
        MLInput.OnControllerButtonUp += OnButtonUp;
        _controller = MLInput.GetController(MLInput.Hand.Left);

        // hand setup
        gestures = new MLHandKeyPose[3]; //Assign the gestures we will look for.
        gestures[0] = MLHandKeyPose.Thumb;
        gestures[1] = MLHandKeyPose.Ok;
        gestures[2] = MLHandKeyPose.OpenHandBack;
        MLHands.KeyPoseManager.EnableKeyPoses(gestures, true, false);

    }

    private void Start()
    {
        focus = converter;
        target.SetActive(false);
        focus.transform.localPosition = transform.position + transform.forward * _distance;
        focus.transform.localRotation = Quaternion.Euler(Vector3.up);
    }

    void OnDestroy()
    {
        MLInput.OnControllerButtonDown -= OnButtonDown;
        MLInput.OnControllerButtonUp -= OnButtonUp;
        MLInput.Stop();
        MLHands.Stop();
    }

    void Update()
    {
        // move focus
        pointer.transform.position = _controller.Position;
        pointer.transform.rotation = _controller.Orientation;
        if (focus == target)
        {
            Plane plane = new Plane(Vector3.up, converter.transform.localPosition);
            Ray pointerRay = new Ray(pointer.transform.position, pointer.transform.forward);
            float enter = 0.0f;
            if (plane.Raycast(pointerRay, out enter))
            {
                //Get the point that is clicked
                Vector3 hitPoint = pointerRay.GetPoint(enter);
                // set the position
                focus.transform.position = hitPoint;
            }
        }

        // swipe rotate
        if (_controller.TouchpadGesture.Type == MLInputControllerTouchpadGestureType.RadialScroll
            && _controller.TouchpadGestureState != MLInputControllerTouchpadGestureState.End)
        {
            float radialScrollSpeed = _controller.TouchpadGesture.Speed;
            float radialDirection = _controller.TouchpadGesture.Direction == MLInputControllerTouchpadGestureDirection.Clockwise ? 1 : -1;
            focus.transform.Rotate(Vector3.up, _rotationSpeed * Time.deltaTime * radialScrollSpeed * radialDirection);
        }

        // pubish target location
        if (_bumper && focus == target && Time.time - TimeLastPublished > DebounceDuration)
        {
            goalPublisher.UpdateMessage();
            TimeLastPublished = Time.time;
            AudioSource.Play();
        }

        // publish location estimate
        if (_controller.TriggerValue > 0.2f && focus == target && Time.time - TimeLastPublished > DebounceDuration)
        {
            locationEstimatePublisher.UpdateMessage();
            TimeLastPublished = Time.time;
            AudioSource.Play();
        }
        
        // leave calibration stage
        if (GetGesture(MLHands.Left, MLHandKeyPose.Ok) || GetGesture(MLHands.Right, MLHandKeyPose.Ok))
        {
            if (focus == converter)
            {
                target.SetActive(true);
                focus = target;
                AudioSource.Play();
            }
        }
    }

    void OnButtonDown(byte controller_id, MLInputControllerButton button)
    {
        if ((button == MLInputControllerButton.Bumper))
        {
            _bumper = true;
        }
    }

    void OnButtonUp(byte controller_id, MLInputControllerButton button)
    {
        if (button == MLInputControllerButton.HomeTap && focus == converter)
        {
            focus.transform.localPosition = transform.position + transform.forward * _distance;
        }
        else if (button == MLInputControllerButton.Bumper)
        {
            _bumper = false;
        }
    }

    bool GetGesture(MLHand hand, MLHandKeyPose type)
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

}