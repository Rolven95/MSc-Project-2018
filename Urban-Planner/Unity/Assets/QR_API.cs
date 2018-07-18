using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;



public class QR_API : MonoBehaviour {

    private static class QRAPI
    {
        [DllImport("HandTrackerLib")]
        public static extern void initEnv();

        [DllImport("HandTrackerLib")]
        public static extern void destroyEnv();

        [DllImport("HandTrackerLib")]
        public static extern void updateSensor();

        [DllImport("HandTrackerLib")]
        public static extern void updateProcessor();

        [DllImport("HandTrackerLib")]
        public static extern bool getColorFrame(ref IntPtr frameData, ref int frameLength);

        [DllImport("HandTrackerLib")]
        public static extern bool getQRResult(ref int[][] qrresult);


        public static byte[] currentColorData = null; // BGR frame stored as 32 bit integers
        public static Color[] currentColorFrame = null; // The color frame for Unity to use
        public static int[][] qrresult = null; // the results from QRCode detected.

        public static bool QueryCurrentColorFrame()
        {
            IntPtr ptrFrame = IntPtr.Zero;
            int frameLen = 0;

            bool success = getColorFrame(ref ptrFrame, ref frameLen);

            //Debug.Log("Color frame len is : " + frameLen);

            if (success)
            {
                if (currentColorData == null)
                    currentColorData = new byte[frameLen];


                Marshal.Copy(ptrFrame, currentColorData, 0, frameLen);


                //Change the BGRA channels to RGBA
                byte[] temp = null;
                temp = new byte[frameLen/4];
                for (int i = 0; i < frameLen/4; i++)
                {
                   
                    temp[i] = currentColorData[i * 4 ];
                    currentColorData[i * 4] = currentColorData[i * 4 + 2];
                    currentColorData[i * 4 + 2] = temp[i];

                }


            }

            return success;
        }
    }

    void Awake()
    {
        QRAPI.initEnv();
    }

    void OnApplicationQuit()
    {
        QRAPI.destroyEnv();
    }

    private Texture2D cameraFrame;

    //This is the UI reference
    public GameObject Plane;

    // Use this for initialization
    void Start ()
    {
        cameraFrame = new Texture2D(1280, 720, TextureFormat.RGBA32, false);
        if (QRAPI.qrresult == null)
            QRAPI.qrresult = new int[30][];
    }
	
	// Update is called once per frame
	void Update ()
    {
        // Update the sensor and underlying 
        QRAPI.updateSensor();
        QRAPI.updateProcessor();

        //IntPtr ptrFrame = IntPtr.Zero;
        //int frameLen = 0;

        //bool success = QRAPI.getColorFrame(ref ptrFrame, ref frameLen);

        bool success = QRAPI.QueryCurrentColorFrame();

        if (success)
        {
           
            // If we have a frame, set the texture map to the frame data and display it
         
            //cameraFrame.SetPixels(QRAPI.currentColorFrame);
            cameraFrame.LoadRawTextureData(QRAPI.currentColorData);
            //cameraFrame.LoadRawTextureData(ptrFrame, frameLen);
       
            cameraFrame.Apply();
            Plane.GetComponent<Renderer>().material.mainTexture = cameraFrame;
            //Debug.Log(cameraFrame.format);
        }


        if (QRAPI.getQRResult(ref QRAPI.qrresult))
        {
            Debug.Log(QRAPI.qrresult);
        }

    }
}
