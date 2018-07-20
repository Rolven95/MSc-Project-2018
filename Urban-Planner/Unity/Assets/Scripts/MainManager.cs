using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

//struct ObjectInfomation
//{
//     int id;
//     int x;
//     int y;
//     int z;
//     int r;
//};

public class MainManager : MonoBehaviour
{
    public const int TotalNumber = 1;

    public GameObject[] Object_List = new GameObject[TotalNumber]; 
    //// 0-4
    //public GameObject Rock0;//0
    //public GameObject Rock1;
    //public GameObject Rock2;
    //public GameObject Rock3;
    //public GameObject Rock4;//4
    ////5-9
    //public GameObject Tree0;//5
    //public GameObject Tree1;
    //public GameObject Tree2;
    //public GameObject Tree3;
    //public GameObject Tree4;//9

    //public GameObject lamppost0;//10
    //public GameObject lamppost1;
    //public GameObject lamppost2;
    //public GameObject lamppost3;//13

    //public GameObject bench0;//14
    //public GameObject bench1;
    //public GameObject bench2;
    //public GameObject bench3;
    //public GameObject bench4;//18
 
    int[][] allinfo = new int[30][]; // [0]id,[1]x,[2]y,[3]z,[4]rotation, [5]lifetime 
   
        

    float [][] CurrentLocation = new float[30][]; //x,y,z,r

    //ObjectInfomation[] AllCurrentLocation = new ObjectInfomation[TotalNumber];

    public Space rotateSpace;

    void Start()
    {
        //this should be where load all location into the CurrentLocation[][]. Use CurrentLocation as a mapping table. 

        for (int i =0; i< TotalNumber; i++) {
            CurrentLocation[i] = new float[4];
            allinfo[i] = new int[6];
           
            CurrentLocation[i][0]= Object_List[i].transform.position.x;
            CurrentLocation[i][1]= Object_List[i].transform.position.y;
            CurrentLocation[i][2]= Object_List[i].transform.position.z;

            CurrentLocation[i][3]= Object_List[i].transform.rotation.y;

            allinfo[i][0] = 99;
            allinfo[i][1] = 0;
            allinfo[i][2] = 0;
            allinfo[i][3] = 0;
            allinfo[i][4] = 0;

        }
        for (int i = TotalNumber; i < allinfo.Length; i++) {
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

        //Debug.Log("Distance:");
        QR_Reader(allinfo);

        Location_Updater(CurrentLocation, allinfo);

        Object_Mover(CurrentLocation);



    }

    int[,] qrresult = new int[30, 6];

    public void QR_Reader(int[][] newdata) { // get image and read QR codes. Put infomation in the allinfo[][]

        newdata[0][0] = 0;
        newdata[0][1] = 80;
        newdata[0][2] = 0;
        newdata[0][3] = 30;
        newdata[0][4] = 300;


        // Update the sensor and underlying 
        //QRAPI.updateSensor();
        //QRAPI.updateProcessor();

        //// Update the result info from Kinect API
        //if (QRAPI.getQRResult(qrresult))
        //{
        //    for (int i = 0; i < 30; i++)
        //    {
        //        if (qrresult[i, 0] != 99 && qrresult[i, 0] >= 0)
        //        {
        //            for (int j = 0; j < 6; j++)
        //            {
        //                //newdata[i][j] = qrresult[i,j];
        //            }
        //        }
        //    }
        //}

        //for (int i = 0; i < TotalNumber; i++)
        //{
        //    newdata[i][0] = 0;
        //    for (int j = 1; j < 6; j++)
        //    {
        //        newdata[i][j] = newdata[i][j] + 1;
        //    }
        //}

        return; 
    }

    public void Location_Updater(float[][] Current, int[][]QRinfo) { // put data into CurrentLocation[][], repackage data
        Debug.Log("entered Location_Updater");
        for (int i = 0; i < TotalNumber; i++)
        {

            Debug.Log("QRinfo[i][0]: " + QRinfo[i][0]);
            if (QRinfo[i][0] != 99)
            {
                Current[QRinfo[i][0]][0] = QRinfo[i][1];
                Current[QRinfo[i][0]][1] = QRinfo[i][2];
                Current[QRinfo[i][0]][2] = QRinfo[i][3];
                Current[QRinfo[i][0]][3] = QRinfo[i][4];
                Debug.Log("Updated");
            }
            Debug.Log("Checked AllInfolist");
        }
        return;
    }

     
    public void Object_Mover(float[][] Current){ //use new locations to move objects  [id][x,y,z,rotation]

        for (int i = 0; i < TotalNumber; i++) {
            change_position(Current[i], Object_List[i]);
            Debug.Log("Moved");
        }

        return;
    }


    public bool Check_List(int id) {
        if (id > 0 && id < TotalNumber)
            return true;
        else
            return false; 
    }



    void change_position(float[] Current, GameObject Objects) // 
    {
        Objects.transform.position = new Vector3(Objects.transform.position.x /100*99 +  Current[0]/100, Objects.transform.position.y / 100 * 99 +  Current[1]/100, Objects.transform.position.z / 100 * 99 + Current[2]/100);

        if (Objects.transform.rotation.y != Current[3])
           // Objects.transform.Rotate(0, (1), 0, Space.World);
            Objects.transform.Rotate(0, ((Current[3]-180)/180-Objects.transform.rotation.y), 0, Space.World);
        Debug.Log("Objects.transform.rotation.y:"+ Objects.transform.rotation.y);
        //Objects.transform.Rotate(new Vector3(0, Current[3] - Objects.transform.rotation.x, 0), rotateSpace);

        //Debug.Log("position changed");
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
        //QRAPI.initEnv();
    }

    void OnApplicationQuit()
    {
        //QRAPI.destroyEnv();
    }
}