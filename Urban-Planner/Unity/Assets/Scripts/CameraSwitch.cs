using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraSwitch : MonoBehaviour
{

    public GameObject cameramain;
    public GameObject cameratop;
    public GameObject cameravive;
    

    //AudioListener cameraOneAudioLis;
    //AudioListener cameraTwoAudioLis;

    // Use this for initialization
    void Start()
    {

        //Get Camera Listeners
        //cameraOneAudioLis = cameraOne.GetComponent<AudioListener>();
        //cameraTwoAudioLis = cameraTwo.GetComponent<AudioListener>();

        //Camera Position Set
        cameraPositionChange(PlayerPrefs.GetInt("CameraPosition"));
    }

    // Update is called once per frame
    void Update()
    {
        //Change Camera Keyboard
        switchCamera();
    }

    //UI JoyStick Method
    public void cameraPositonM()
    {
        cameraChangeCounter();
    }

    //Change Camera Keyboard
    void switchCamera()
    {
        if (Input.GetKeyDown(KeyCode.C) || Input.GetKeyDown(KeyCode.LeftAlt) || Input.GetKeyDown(KeyCode.RightAlt))
        {
            cameraChangeCounter();
        }
    }

    //Camera Counter
    void cameraChangeCounter()
    {
        int cameraPositionCounter = PlayerPrefs.GetInt("CameraPosition");
        cameraPositionCounter++;
        cameraPositionChange(cameraPositionCounter);
    }

    //Camera change Logic
    void cameraPositionChange(int camPosition){
    
        if (camPosition > 2)
        {
            camPosition = 0;
        }

        //Set camera position database
        PlayerPrefs.SetInt("CameraPosition", camPosition);

        //Set camera position 1
        if (camPosition == 0)
        {
            cameramain.SetActive(true);
            //cameraOneAudioLis.enabled = true;

            //cameraTwoAudioLis.enabled = false;
            cameratop.SetActive(false);
            cameravive.SetActive(false);
        }

        //Set camera position 2
        if (camPosition == 1)
        {
            cameramain.SetActive(false);
            //cameraOneAudioLis.enabled = true;

            //cameraTwoAudioLis.enabled = false;
            cameratop.SetActive(true);
            cameravive.SetActive(false);
        }
        if(camPosition == 2){


            cameramain.SetActive(false);
            //cameraOneAudioLis.enabled = true;

            //cameraTwoAudioLis.enabled = false;
            cameratop.SetActive(false);
            cameravive.SetActive(true);

        }
    }
}