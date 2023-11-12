using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LevParticle : MonoBehaviour
{
    private Vector3 particlePos;
    public bool selected;

    void Start()
    {
        particlePos = this.transform.position;
        selected = false;
    }

    void Update()
    {
        if(this.selected){
            // Debug.Log(this.gameObject.GetInstanceID());
        }   
    }

    public void MoveX(int dir)
    {
        this.transform.position += new Vector3(dir * 0.1F, 0, 0);
    }

    public void MoveY(int dir)
    {
        this.transform.position += new Vector3(0, 0, dir * 0.1F);
    }

    public void MoveZ(int dir)
    {
        this.transform.position += new Vector3(0, dir * 0.1F, 0);
    }

    public void DeleteParticle()
    {
        Destroy(gameObject);
    }
}
