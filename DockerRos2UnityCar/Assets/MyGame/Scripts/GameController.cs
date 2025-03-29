using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class GameController : MonoBehaviour {

    private TimeCount timeCount;
     public  Button button;
    public GameObject bu;



    void Start() {
        timeCount = GetComponent<TimeCount>();//获取TimeCount
        //button.enabled = false;
        bu.GetComponent<CanvasGroup>().alpha = 0;
        bu.GetComponent<CanvasGroup>().interactable = false;
        bu.GetComponent<CanvasGroup>().blocksRaycasts = false;
    }

    void OnTriggerEnter(Collider other) {
        if (other.transform.root.tag == "Car") {
            timeCount.Stop();//计时停止
            bu.GetComponent<CanvasGroup>().alpha = 1;
            bu.GetComponent<CanvasGroup>().interactable = true;
            bu.GetComponent<CanvasGroup>().blocksRaycasts = true;
        }
    }
    public void onClick0()
    {
        SceneManager.LoadScene(0);
    }
    public void onClick1()
    {
        SceneManager.LoadScene(1);
    }

}
