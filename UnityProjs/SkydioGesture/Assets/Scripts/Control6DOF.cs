using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;

public class Control6DOF : MonoBehaviour {

    #region Private Variables
    private MLInputController controller;
    #endregion

    #region Public Variables
    #endregion

    #region Unity Methods
    void Start () {
        //Start receiving input by the Control
        MLInput.Start();
        controller = MLInput.GetController(MLInput.Hand.Left);
	}

    void OnDestroy()
    {
        //Stop receiving input by the Control
        MLInput.Stop();    
    }

    void Update () {
        transform.position = controller.Position;
        transform.rotation = controller.Orientation;
	}
    #endregion
}