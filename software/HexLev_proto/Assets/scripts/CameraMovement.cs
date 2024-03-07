using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Controls the zoom level by physically moving the camera within the space.
/// The camera moves towards the levitator (as a fixed point).
/// This class is attached to the HexCam camera within Unity.
/// </summary>
public class CameraMovement : MonoBehaviour
{
    /// <summary>
    /// Stores the camera position as 3D-coordinates
    /// </summary>
    private Vector3 CameraPosition;

    /// <summary>
    /// GameObject Transform to act as an anchor point for the camera. A dummy object for the centre of the levitator.
    /// </summary>
    public Transform DummyObject;

    /// <summary>
    /// Specifies the speed at which the camera moves. Provided in the Unity editor.
    /// </summary>
    public float positionSpeed = 0.1F;

    /// <summary>
    /// Specifies how close the camera can move towards the dummy (levitator centre).
    /// </summary>
    public int maxZoom = 90;

    /// <summary>
    /// Specifies how far the camera can move away from the dummy (levitator centre).
    /// </summary>
    public int minZoom = 23;

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

        if ((CameraPosition - DummyObject.position).magnitude >= maxZoom || (CameraPosition - DummyObject.position).magnitude <= minZoom)
        {
            CameraPosition = this.transform.position;
        }

        this.transform.position = CameraPosition;
        this.transform.LookAt(DummyObject);

    }
}
