using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HeadPose : MonoBehaviour {

    public Transform cameraTransform;
	
	// Update is called once per frame
	void Update () {
        transform.position = cameraTransform.position;
        transform.rotation = cameraTransform.rotation;
	}
}
