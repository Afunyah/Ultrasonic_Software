using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveParticle : MonoBehaviour
{

    private GameObject selectedParticle;
    public int dir;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        selectedParticle = GameObject.FindWithTag("SELECTED");
    }

    public void MoveX()
    {
        if (selectedParticle == null)
        {
            return;
        }
        selectedParticle.transform.position += new Vector3(dir*0.1F, 0, 0);
    }

    public void MoveY()
    {
        if (selectedParticle == null)
        {
            return;
        }
        selectedParticle.transform.position += new Vector3(0, 0, dir*0.1F);
    }

    public void MoveZ()
    {
        if (selectedParticle == null)
        {
            return;
        }
        selectedParticle.transform.position += new Vector3(0, dir*0.1F, 0);
    }
}
