using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainManager : MonoBehaviour
{
    public const int TotalNumber = 19;

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
 
    int[][] allinfo = new int[5][]; // [0]id,[1]x,[2]y,[3]z,[4]rotation, [5]lifetime 

    float [][] CurrentLocation = new float[5][];
   
    void Start()
    {
        //this should be where load all location into the CurrentLocation[][]. Use CurrentLocation as a mapping table. 
        

    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log("Distance:");
        QR_Reader(allinfo);

        Location_Updater(CurrentLocation, allinfo);

        Object_Mover(CurrentLocation);
    }

    

    public void QR_Reader(int[][] newdata) { // get image and read QR codes. Put infomation in the allinfo[][]
        
        return; 
    }

    public void Location_Updater(float[][] Current, int[][]QRinfo) { // put data into CurrentLocation[][], repackage data


        return;
    }


    public void Object_Mover(float[][] Current){ //use new locations to move objects  [id][x,y,z,rotation]

        for (int i = 0; i < TotalNumber; i++) {

            change_position(Current[i], Object_List[i]);

        }

        return;
    }


    public bool Check_List(int id) {
        if (id > 0 && id < TotalNumber)
            return true;
        else
            return false; 
    }



    void change_position(float[] Current, GameObject Objects) // this is a example method showing how to move objects. 
    {
        Objects.transform.position = new Vector3(Objects.transform.position.x /10*9 +  Current[0]/10, Objects.transform.position.y / 10 * 9 +  Current[1]/10, Objects.transform.position.z / 10 * 9 + Current[2]/10);

        Objects.transform.rotation = Quaternion.Euler(new Vector3(Current[3], 0, 0));
        //Objects.transform.position.y * 0.9 + 0.1* Current[1], Objects.transform.position.z * 0.9 + Current[2])
        //if (Rock0.transform.position.z < 10)
        //{
        //    //Debug.Log("Moving the rock");
        //    Rock0.transform.position = Vector3.Lerp(Rock0.transform.position, new Vector3(10, 10, 10), Time.deltaTime);
        //    Debug.Log(Rock0.transform.position);
        //}
        //Rock0.transform.position = new Vector3 (10.0f, 10.0f, 10.0f);
        Debug.Log("Update called");
    }
}