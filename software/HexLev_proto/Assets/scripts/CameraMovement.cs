using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{   
    private Vector3 CameraPosition;
    public Transform DummyObject;
    public float positionSpeed = 0.1F;

    void Start()
    {        

    }

    void Update()
    {       
        CameraPosition = this.transform.position; 
        // if(Input.GetKey(KeyCode.W)){
        //     CameraPosition = Vector3.MoveTowards(this.transform.position, transform.parent.position, positionSpeed);
        // }
        // else if(Input.GetKey(KeyCode.S)){
        //     CameraPosition = Vector3.MoveTowards(this.transform.position, transform.parent.position, -positionSpeed);
        // }

        CameraPosition = Vector3.MoveTowards(this.transform.position, transform.parent.position, Input.mouseScrollDelta.y);
        
        if((CameraPosition - DummyObject.position).magnitude > 100){
            CameraPosition = this.transform.position;
        }

        this.transform.position = CameraPosition;
        this.transform.LookAt(DummyObject);
        
    }
}
