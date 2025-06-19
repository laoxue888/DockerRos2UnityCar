using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lightof : MonoBehaviour {

    // Use this for initialization
    public MeshRenderer on;

	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            on.enabled = false;
        }
        if (Input.GetKeyUp(KeyCode.Space))
        {
            on.enabled = true;
        }
    }
}
