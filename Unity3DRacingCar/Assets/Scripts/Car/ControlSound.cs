using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ControlSound : MonoBehaviour {

    // Use this for initialization
    public AudioSource bg;
    
    public Slider sd;

    public void conSound()
    {
        bg.volume = sd.value;
        
    }
}
