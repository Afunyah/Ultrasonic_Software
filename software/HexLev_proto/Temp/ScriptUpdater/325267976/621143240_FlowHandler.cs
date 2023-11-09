using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlowHandler : MonoBehaviour
{
    // private Renderer rend;
    // public new Camera camera;

    void Update(){
        
        if(Input.GetMouseButton(0)){
            Ray ray = GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
            if(Physics.Raycast(ray, out RaycastHit hitInfo)){
                if(hitInfo.collider.gameObject.GetComponent<SelectParticle>() != null){
                    rend = hitInfo.collider.gameObject.GetComponent<Renderer>();
                    rend.material.color = Color.red;
                    
                }
            }
        }


    }
}
