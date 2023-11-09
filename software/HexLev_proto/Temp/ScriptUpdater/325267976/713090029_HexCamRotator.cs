using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HexCamRotator : MonoBehaviour
{

    public float rotationSpeed = 1;

    void Start()
    {
        
    }

    void Update()
    {
        if(Input.GetMouseButton(0)){
            // transform.Rotate(rotationSpeed*-Input.GetAxis("Mouse Y"), rotationSpeed*Input.GetAxis("Mouse X"),0);
            Ray ray = GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
            Debug.Log(ray);
        }
        

    }
}
