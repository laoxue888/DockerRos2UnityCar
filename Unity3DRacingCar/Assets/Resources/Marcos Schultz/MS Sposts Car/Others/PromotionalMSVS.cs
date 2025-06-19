using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PromotionalMSVS : MonoBehaviour {

	void OnGUI(){
		if (GUI.Button (new Rect (20, 20, 400, 50), "Need a vehicle controller? CLICK HERE")) {
			Application.OpenURL ("https://assetstore.unity.com/packages/tools/physics/ms-vehicle-system-vehicle-controller-88035");
		}
	}
}
