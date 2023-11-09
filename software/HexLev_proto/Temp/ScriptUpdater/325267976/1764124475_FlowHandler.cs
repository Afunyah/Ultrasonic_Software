using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class FlowHandler : MonoBehaviour
{
    void update(){
        
        if (Input.GetMouseButtonDown(0)){
            Ray ray = GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);

            if(Physics.Raycast(ray, out RaycastHit hitInfo)){
                if(hitInfo.collider.gameObject.GetComponent<selectedParticle>() != null){
                    Renderer rend = hitInfo.collider.gameObject.GetComponent<renderer>();
                    rend.material.color = Color.Red;
                }
            }
        }

    }
}
