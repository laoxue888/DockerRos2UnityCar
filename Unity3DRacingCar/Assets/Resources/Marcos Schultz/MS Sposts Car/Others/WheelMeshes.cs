using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[Serializable]
public class WheelInstance{
	public WheelCollider _wheelCollider;
	public GameObject _wheelMesh;
}

public class WheelMeshes : MonoBehaviour {

	public WheelInstance[] _vehicleWheels;

	void Update () {
		for (int x = 0; x < _vehicleWheels.Length; x++) {
			if (_vehicleWheels [x]._wheelCollider && _vehicleWheels [x]._wheelMesh) {
				Vector3 tempPos;
				Quaternion tempRot;
				_vehicleWheels [x]._wheelCollider.GetWorldPose (out tempPos, out tempRot);
				_vehicleWheels [x]._wheelMesh.transform.position = tempPos;
				_vehicleWheels [x]._wheelMesh.transform.rotation = tempRot;
			}
		}
	}
}
