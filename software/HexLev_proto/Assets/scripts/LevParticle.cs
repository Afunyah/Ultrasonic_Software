using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class LevParticle : MonoBehaviour
{
    private Vector3 particlePos;
    private bool selected;
    private List<Transducer> nearbyTransducers;


    void Start()
    {
        nearbyTransducers = new List<Transducer> { };
        particlePos = this.transform.position;
        selected = false;
    }

    void Update()
    {
        foreach (Transducer x in nearbyTransducers)
        {
            Debug.Log(x.name);
        }

    }

    public List<Transducer> findNearbyTransducers()
    {

        return nearbyTransducers;
    }

    private void OnTriggerStay(Collider other)
    {
        // Debug.Log(other.gameObject.name);
        Transducer trs = other.gameObject.GetComponent<Transducer>();

        if (trs != null && !trs.IsActive())
        {
            trs.Activate();
            nearbyTransducers.Add(trs);
        }

    }

    private void OnTriggerExit(Collider other)
    {
        // Debug.Log(other.gameObject.name);
        Transducer trs = other.gameObject.GetComponent<Transducer>();

        if (trs != null)
        {
            trs.Deactivate();
            nearbyTransducers.Remove(trs);
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

    public void SetSelect(bool sel)
    {
        selected = sel;
    }

    public void DeleteParticle()
    {
        Destroy(gameObject);
    }
}
