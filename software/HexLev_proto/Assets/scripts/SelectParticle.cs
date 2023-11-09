using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectParticle : MonoBehaviour
{
    public Material selectedMaterial;
    private new Renderer renderer;
    // Start is called before the first frame update
    void Start()
    {
        renderer = this.GetComponent<Renderer>(); 
    }

        public void mdh(){
        Debug.Log("dafs");
    }


    // Update is called once per frame
    void Update()
    {
    }

    // private void OnMouseEnter(){
    //     // renderer.material.color = Color.red;
    // }

    // private void OnMouseDown(){
    //     // renderer.material.color = Color.red;
    //     // renderer.material = selectedMaterial;
    // }

    
}
