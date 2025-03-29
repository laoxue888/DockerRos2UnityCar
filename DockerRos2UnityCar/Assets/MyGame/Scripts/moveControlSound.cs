using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class moveControlSound : MonoBehaviour {

    // Use this for initialization
    public AudioSource move;

    public Slider sd;
    public void conSound()
    {
        move.volume = sd.value;

    }
}
