using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;

public class POMDPEyeTracker : MonoBehaviour {

    public GameObject Camera;
    public GameObject RaycastHitPoint;
    private Vector3 _heading;


    void Start () {
        MLEyes.Start();
    }

    private void OnDestroy()
    {
        MLEyes.Stop();
    }

    void Update()
    {
        if (MLEyes.IsStarted && !float.IsNaN(MLEyes.FixationPoint.x))
        {
            RaycastHit rayHit;
            _heading = MLEyes.FixationPoint - Camera.transform.position;
            if (Physics.Raycast(Camera.transform.position, _heading, out rayHit, 100.0f))
            {
                RaycastHitPoint.transform.position = rayHit.point;
            } else
            {
                RaycastHitPoint.transform.position = MLEyes.FixationPoint;
            }
        }
    }
}
