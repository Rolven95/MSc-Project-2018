using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rock_group2 : MonoBehaviour {

    public GameObject Rock2;
    // Use this for initialization
    void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

        this.transform.position = new Vector3(this.transform.position.x + 1, this.transform.position.y, this.transform.position.z);

    }
}
