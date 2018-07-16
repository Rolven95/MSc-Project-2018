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

        public static byte[] currentColorData = null; // BGR frame stored as 32 bit integers
        public static Color[] currentColorFrame = null; // The color frame for Unity to use

        public static bool QueryCurrentColorFrame()
        {
            IntPtr ptrFrame = IntPtr.Zero;
            int frameLen = 0;

            bool success = getColorFrame(ref ptrFrame, ref frameLen);

        
            if (success)
            {
                if (currentColorData == null)
                    currentColorData = new byte[frameLen];

                if (currentColorFrame == null)
                    currentColorFrame = new Color[frameLen];

                Marshal.Copy(ptrFrame, currentColorData, 0, frameLen);

                // Now create the color array for texturing
                // Bytes are reversed as the underlying OpenCV implementation uses BGR form
                for (int i = 0, k = 0; i < 512; ++i)
                {
                    for (int j = 0; j <512; ++j, k += 3)
                    {
                        currentColorFrame[j + (i * 512)] = new Color(QRAPI.currentColorData[k + 2], QRAPI.currentColorData[k + 1], QRAPI.currentColorData[k]);
                    }
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
        cameraFrame = new Texture2D(512, 512);
    }
	
	// Update is called once per frame
	void Update ()
    {
        // Update the sensor and underlying 
        QRAPI.updateSensor();
        QRAPI.updateProcessor();

        bool success = QRAPI.QueryCurrentColorFrame();
        if (success)
        {
            // If we have a frame, set the texture map to the frame data and display it
            Plane.GetComponent<Renderer>().material.mainTexture = cameraFrame;
            cameraFrame.SetPixels(QRAPI.currentColorFrame);
            cameraFrame.Apply();
        }

    }
}
