using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeTextScript : MonoBehaviour {

	// Use this for initialization
	void Start () {
        TextMesh textObject = GameObject.Find("text").GetComponent<TextMesh>();
        textObject.text = "test";
		
	}
	
	// Update is called once per frame
	void Update () {
		
	}
}
