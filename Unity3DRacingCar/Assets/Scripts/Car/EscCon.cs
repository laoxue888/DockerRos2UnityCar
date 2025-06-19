using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EscCon : MonoBehaviour {

    // Use this for initialization
    public GameObject sound;
    void Start () {
        sound.GetComponent<CanvasGroup>().alpha = 0;
        sound.GetComponent<CanvasGroup>().interactable = false;
        sound.GetComponent<CanvasGroup>().blocksRaycasts = false;
    }
	
	// Update is called once per frame
	void Update () {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            sound.GetComponent<CanvasGroup>().alpha = 1;
            sound.GetComponent<CanvasGroup>().interactable = true;
            sound.GetComponent<CanvasGroup>().blocksRaycasts = true;
            Time.timeScale = 0;
        }
        if (Input.GetKeyUp(KeyCode.Escape))
        {
            Time.timeScale = 1;
            sound.GetComponent<CanvasGroup>().alpha = 0;
            sound.GetComponent<CanvasGroup>().interactable = false;
            sound.GetComponent<CanvasGroup>().blocksRaycasts = false;
        }
    }
}
