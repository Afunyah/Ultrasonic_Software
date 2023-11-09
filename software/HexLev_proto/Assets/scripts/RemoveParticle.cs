using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RemoveParticle : MonoBehaviour
{
    private GameObject selectedParticle;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void DeleteParticle(){
        selectedParticle = GameObject.FindWithTag("SELECTED");
        if (selectedParticle == null)
        {
            return;
        }
        Destroy(selectedParticle);
    }
}
