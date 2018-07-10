using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainManager : MonoBehaviour
{

    public GameObject Rock0;
    public GameObject Rock1;
    public GameObject Rock2;
    public GameObject Rock3;
    public GameObject Rock4;

    public GameObject Tree0;
    public GameObject Tree1;
    public GameObject Tree2;
    public GameObject Tree3;
    public GameObject Tree4;

    public GameObject lamppost0;
    public GameObject lamppost1;
    public GameObject lamppost2;
    public GameObject lamppost3;

    public GameObject bench0;
    public GameObject bench1;
    public GameObject bench2;
    public GameObject bench3;
    public GameObject bench4;


    
    void Start()
    {
          //Vector3 RockGroup0_Position = Rock0.transform.position;
          //float x = RockGroup0_Position.x;
        

    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log("Distance:");
        change_position();

    }

    void change_position() {

        if (Rock0.transform.position.z < 10)
        {
            //Debug.Log("Moving the rock");
            Rock0.transform.position = Vector3.Lerp(Rock0.transform.position, new Vector3(10, 10, 10), Time.deltaTime);
            Debug.Log(Rock0.transform.position);
        }
        //Rock0.transform.position = new Vector3 (10.0f, 10.0f, 10.0f);
        Debug.Log("Update called");
        //Rock0.transform.position = new Vector3(Rock0.transform.position.x + 1, Rock0.transform.position.y, Rock0.transform.position.z);

        //Vector3 RockGroup0_Position = Rock0.transform.position;
        //Debug.Log(" \n x = "+ RockGroup0_Position.x);
        //RockGroup0_Position.x++;
        //Debug.Log(" x = "+ RockGroup0_Position.x);
        //Rock0.transform.position = RockGroup0_Position;
        //Debug.Log(" Rock x is : " + Rock0.transform.position.x);
    }
    
}