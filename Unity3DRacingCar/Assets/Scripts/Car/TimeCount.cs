using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class TimeCount : MonoBehaviour {

    public Text label;
    public float totalTime = 0;//用时计时

    private bool isStart = false;
	// Use this for initialization
	void Start () {
        label.enabled = false;
        totalTime=0;
	}

    void Update() {
        if (isStart) {
            totalTime += Time.deltaTime;
            label.text = totalTime.ToString();
        }
    }

    public void Show() {
        isStart = true;
        label.enabled = true;
    }
    public void Stop() {
        isStart = false;
    }
}
