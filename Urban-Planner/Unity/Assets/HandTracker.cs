using UnityEngine;
using System.Collections;
using System.IO;
using System.Runtime.InteropServices;
using System;

/**
 * This is the behaviour that interfaces
 * to the low level C++ HandTracker implementation
 *
 * @author Daniel J. Finnegan
 * @date April 2016
 */
public class HandTracker : MonoBehaviour
{
    /// <summary>
    /// Here are the actual functions to the C++ layer.
    /// They should never be called directly, instead the
    /// public methods should be called instead
    /// </summary>
    private static class IHandTracker
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
        public static extern void updateHandTracker();

        [DllImport("HandTrackerLib")]
        public static extern void setResizeProcessorParams(int width, int height);

        [DllImport("HandTrackerLib")]
        public static extern bool getDepthFrame(ref IntPtr frameData, ref int frameLength);

        [DllImport("HandTrackerLib")]
        public static extern bool getContourFrame(ref IntPtr frameData, ref int frameLength);

        // Keep a managed buffer to the depth and contour frames here
        public static short[] currentDepthFrame = null;
        public static byte[] currentContourData = null; // BGR frame stored as 32 bit integers
        public static Color[] currentContourFrame = null; // The color frame for Unity to use

        /// <summary>
        /// The following methods are responsible for marshalling
        /// the depth frame and other data from unmanaged C++ to the Unity layer
        /// </summary>
        /// 

        public static bool QueryCurrentDepthFrame()
        {
            IntPtr ptrFrame = IntPtr.Zero;
            int frameLen = 0;

            bool success = getDepthFrame(ref ptrFrame, ref frameLen);

            //Debug.Log("Depth frame len is : " + frameLen);

            if (success)
            {
                if (currentDepthFrame == null)
                    currentDepthFrame = new short[frameLen];

                Marshal.Copy(ptrFrame, currentDepthFrame, 0, frameLen);
            }

            return success;
        }

        public static bool QueryCurrentContourFrame()
        {
            IntPtr ptrFrame = IntPtr.Zero;
            int frameLen = 0;

            bool success = getContourFrame(ref ptrFrame, ref frameLen);

            Debug.Log("Contour frame len is : " + frameLen);

            if (success)
            {
                if (currentContourData == null)
                    currentContourData = new byte[frameLen];

                if (currentContourFrame == null)
                    currentContourFrame = new Color[frameLen];

                Marshal.Copy(ptrFrame, currentContourData, 0, frameLen);

                // Now create the color array for texturing
                // Bytes are reversed as the underlying OpenCV implementation uses BGR form
                for (int i = 0, k = 0; i < 1280; ++i)
                {
                    for (int j = 0; j < 720; ++j, k += 3)
                    {
                        currentContourFrame[j + (i * 720)] = new Color(IHandTracker.currentContourData[k + 2], IHandTracker.currentContourData[k + 1], IHandTracker.currentContourData[k]);
                    }
                }
            }

            return success;
        }
    }


    /// <summary>
    /// Awake and OnApplicationQuit are used to
    /// init and kill the underlying hand tracker
    /// </summary>

    void Awake()
    {
        IHandTracker.initEnv();
    }

    void OnApplicationQuit()
    {
        IHandTracker.destroyEnv();
    }

    /// <summary>
    /// The remaining functions are standard MonoBehaviour affair
    /// </summary>

    private Texture2D cameraFrame;

    //This is the UI reference
    public GameObject Plane;

    // Use this for initialization
    void Start()
    {
        cameraFrame = new Texture2D(1280, 720);
    }

    // Update is called once per frame
    void Update()
    {
        // Update the sensor and underlying 
        IHandTracker.updateSensor();
        IHandTracker.updateProcessor();
        IHandTracker.updateHandTracker();


        // Now try and get the current depth frame
        bool success = IHandTracker.QueryCurrentContourFrame();
        if (success)
        {
            // If we have a frame, set the texture map to the frame data and display it
            Plane.GetComponent<Renderer>().material.mainTexture = cameraFrame;
            cameraFrame.SetPixels(IHandTracker.currentContourFrame);
            //cameraFrame.LoadImage(IHandTracker.currentContourData, false);
            cameraFrame.Apply();
            Debug.Log(cameraFrame.format);
        }
    }
}
