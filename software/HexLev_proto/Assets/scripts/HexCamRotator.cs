using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Controls the camera angle by physically rotating the camera about the centre of the levitator.
/// </summary>
public class HexCamRotator : MonoBehaviour
{
    /// <summary>
    /// Specifies how quickly the camera can rotate.
    /// </summary>
    public float rotationSpeed = 1;

    void Start()
    {
        
    }

    void Update()
    {
        if(Input.GetMouseButton(0)){
            transform.Rotate(rotationSpeed*-Input.GetAxis("Mouse Y"), rotationSpeed*Input.GetAxis("Mouse X"),0);
        }
        

    }
}
