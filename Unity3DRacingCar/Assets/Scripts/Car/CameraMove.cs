using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMove : MonoBehaviour {
    public Transform target;

    public float height ;
    public float distance ;
    public float smoothSpeed ;


    // Update is called once per frame
    void Update()
    {
        Vector3 targetForward = target.forward;
        targetForward.y = 0;

        Vector3 currentForward = transform.forward;
        currentForward.y = 0;


        Vector3 forward = Vector3.Lerp(currentForward.normalized, targetForward.normalized, smoothSpeed * Time.deltaTime);

        Vector3 targetPos = target.position + Vector3.up * height - forward * distance;
        this.transform.position = targetPos;
        transform.LookAt(target);
    }
}
