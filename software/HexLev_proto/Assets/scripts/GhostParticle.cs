using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GhostParticle : MonoBehaviour
{

    private Vector3 particlePos;
    private List<Transducer> nearbyTransducers;


    void Start()
    {
        nearbyTransducers = new List<Transducer> { };
    }


    void Update()
    {
        foreach (Transducer x in nearbyTransducers)
        {
            // Debug.Log(x.name);
        }
    }

    public List<Transducer> FindNearbyTransducers()
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
}
