using UnityEngine;
using System.Collections;
using UnityEngine.XR.MagicLeap;
public class CaptureOffset : MonoBehaviour {

    #region Public Variables
    public Material captureOffsetMaterial;
    #endregion

	#region Private Variables
    private MLInputController _controller;
	public float _offsetU = 0.0f;
    public float _offsetV = 0.0f;
	#endregion
	
    #region Unity Methods
	void Start () {
        _controller = GameObject.Find("Control").GetComponent<POMDPControl>().controller;
	}

    //void Update() {
    //    if (_controller.Touch1PosAndForce.z > 0.0) {
    //        _offsetU += _controller.Touch1PosAndForce.x;
    //        _offsetV -= _controller.Touch1PosAndForce.y;
    //    }
    //}
	
	void OnRenderImage(RenderTexture srcTexture, RenderTexture destTexture) {
		captureOffsetMaterial.SetFloat("_OffsetU", _offsetU);
		captureOffsetMaterial.SetFloat("_OffsetV", _offsetV);
		Graphics.Blit(srcTexture, destTexture, captureOffsetMaterial);
	}
	#endregion
}
