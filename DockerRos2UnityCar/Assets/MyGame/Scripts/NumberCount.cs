using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class NumberCount : MonoBehaviour {

    public int startCount =3;
    public Text timeCount;//倒计时

    private CarMove driving;

    void Start() {
        driving = GameObject.FindGameObjectWithTag("Car").GetComponent<CarMove>();
        StartCoroutine(Count());
    }
	

    IEnumerator Count() {
        timeCount.text = startCount.ToString();
        while (startCount > 0) {
            yield return new WaitForSeconds(1f);//一秒计时
            startCount--;
            timeCount.text = startCount.ToString();
        }
        yield return new WaitForSeconds(0.01f);
        driving.enabled = true;
        timeCount.enabled = false;       
        this.GetComponent<TimeCount>().Show();
    }
}
