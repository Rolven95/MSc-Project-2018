using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

public class MainManager : MonoBehaviour
{
    public const int TotalNumber = 3;

    public int[] TemCurrentDegree;
    
    public GameObject[] Object_List = new GameObject[TotalNumber]; 

    int[][] allinfo = new int[30][]; // [0]id,[1]x,[2]y,[3]z,[4]rotation, [5]lifetime 
   
    float [][] CurrentLocation = new float[TotalNumber][]; //x,y,z,r

    public Space rotateSpace;

   // public DateTime[] TimeList;
    //public int[] TotalTime;
    //public int FrameCounter;
    
    void Start()
    {
       // TimeList = new DateTime[4];
       // TotalTime = new int[4];
        //FrameCounter = 0; 

        //this should be where load all location into the CurrentLocation[][]. Use CurrentLocation as a mapping table. 
        TemCurrentDegree = new int[TotalNumber];
        
        //Debug.Log("CurrentLength(3)="+ CurrentLocation.Length);
        for (int i =0; i< CurrentLocation.Length; i++) {
            
            CurrentLocation[i] = new float[4];

            CurrentLocation[i][0]= Object_List[i].transform.position.x;
            CurrentLocation[i][1]= Object_List[i].transform.position.y;
            CurrentLocation[i][2]= Object_List[i].transform.position.z;
            CurrentLocation[i][3]= 0; // differ of current  and reading angle 

        }
        for (int i = 0; i < allinfo.Length; i++)
        {
            allinfo[i] = new int[6];
            allinfo[i][0] = 99;
            allinfo[i][1] = 0;
            allinfo[i][2] = 0;
            allinfo[i][3] = 0;
            allinfo[i][4] = 0;
        }
    }

    // Update is called once per frame
    void Update()
    {


      //  FrameCounter++;
        //Debug.Log("Distance:");
      //  TimeList[0] = System.DateTime.Now; 
        QR_Reader(allinfo);
       // TimeList[1] = System.DateTime.Now;
        Location_Updater(CurrentLocation, allinfo);
        //TimeList[2] = System.DateTime.Now;
        Object_Mover(CurrentLocation);
       // TimeList[3] = System.DateTime.Now;
        //if (FrameCounter >= 500)
        //    Debug.Log(TotalTime[0]+" "+TotalTime[1] + " "+ TotalTime[2] + " "+ TotalTime[3] );
        //TotalTime[0] += (TimeList[1] - TimeList[0]).Milliseconds;
        //TotalTime[1] += (TimeList[2] - TimeList[1]).Milliseconds;


    }

    int[,] qrresult = new int[30, 6];

    public void QR_Reader(int[][] newdata) { // get image and read QR codes. Put infomation in the allinfo[][]
        
        //Update the sensor and underlying
        QRAPI.updateSensor();
        QRAPI.updateProcessor();

        // Update the result info from Kinect API
        if (QRAPI.getQRResult(qrresult))
        {
            for (int i = 0; i < TotalNumber; i++)
            {
                if(qrresult[i, 0] != 99)
                 //Debug.Log("ID:" + qrresult[i, 0]);

                if (qrresult[i, 0] != 99 && Check_range(qrresult[i, 0]))
                {
                    newdata[i][0] = qrresult[i, 0];
                    newdata[i][1] = -1 * (qrresult[i, 1] / 6 - 50);
                    newdata[i][2] = 0;
                    newdata[i][3] = -1*(qrresult[i, 2] / 4 - 50);
                    newdata[i][4] = qrresult[i, 4];
                    Debug.Log("ID:" + qrresult[i, 0] + " " + qrresult[i, 1] + " " + qrresult[i,2] + " " + qrresult[i, 3] + " " + qrresult[i, 4] +" "+qrresult[i, 5]);
                    //Debug.Log("ID:"+newdata[i][0]+" " +newdata[i][1] + " " + newdata[i][2] + " " + newdata[i][3] + " " + newdata[i][4]);
                    }

            }
        }
        return; 
    }

    public void Location_Updater(float[][] Current, int[][]QRinfo) { // put data into CurrentLocation[][], repackage data
        //Debug.Log("entered Location_Updater");
        for (int i = 0; i < TotalNumber; i++)
        {
            
            if (QRinfo[i][0] != 99)
            {
                //Debug.Log("QRinfo[i][0]: " + QRinfo[i][0]);
                //Debug.Log("i=" + i);
                Current[QRinfo[i][0]][0] = QRinfo[i][1];
                Current[QRinfo[i][0]][1] = QRinfo[i][2];
                Current[QRinfo[i][0]][2] = QRinfo[i][3];
                Current[QRinfo[i][0]][3] = QRinfo[i][4] - TemCurrentDegree[i];

                TemCurrentDegree[i] = QRinfo[i][4];
                //Debug.Log("Updated");
            }
            //Debug.Log("Checked AllInfolist");
        }
        return;
    }
     
    public void Object_Mover(float[][] Current){ //use new locations to move objects  [id][x,y,z,rotation]

        for (int i = 0; i < TotalNumber; i++) {
            change_position(Current[i], Object_List[i]);
            //Debug.Log("Moved");
        }
        return;
    }
    void change_position(float[] Current, GameObject Objects) // 
    {
        Objects.transform.position = new Vector3(Objects.transform.position.x /2 +  Current[0]/2, Objects.transform.position.y / 2+  Current[1]/2, Objects.transform.position.z /2 + Current[2]/2);
        Objects.transform.Rotate(new Vector3(0, Current[3], 0), rotateSpace);
        //Debug.Log("position changed");
    }
    bool Check_range(int id) {
        if (id>=0 && id <TotalNumber) 
            return true;

        return false;
    }
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
        public static extern bool getQRResult(int[,] qrresult);

        [DllImport("HandTrackerLib")]
        public static extern bool testFun(int[] array);
    }

    void Awake()
    {
        QRAPI.initEnv();
    }

    void OnApplicationQuit()
    {
        QRAPI.destroyEnv();
    }
}