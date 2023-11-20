using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

public class GhostParticle : MonoBehaviour
{

    private Vector3 particlePos;
    private List<Transducer> NearbyTransducers;
    private float ColliderRadius;

    void Awake(){
        NearbyTransducers = new List<Transducer>();
        ColliderRadius = this.GetComponent<SphereCollider>().radius*this.transform.localScale.x;
        particlePos = this.transform.position;
        this.gameObject.layer = LayerMask.NameToLayer("TransducerLayer");
    }

    void Start()
    {
    }


    void Update()
    {
        foreach (Transducer x in NearbyTransducers)
        {
            // Debug.Log(x.name);
        }
    }

    public Vector3 GetPostion()
    {
        return this.particlePos;
    }

    public Vector2 GetXYPosition()
    {
        return new Vector2(this.particlePos.x, this.particlePos.z);
    }

    public List<Transducer> FindNearbyTransducers()
    {
        Collider[] forcedCollisions = Physics.OverlapSphere(this.transform.position, ColliderRadius);
        foreach (Collider hitColliders in forcedCollisions)
        {
            // this.gameObject.SendMessage("OnTriggerStay", hitColliders);
            if(hitColliders.GetComponent<Transducer>()!=null){
                Transducer trs = hitColliders.GetComponent<Transducer>();
                this.NearbyTransducers.Add(trs);
            }
        }
        
        
        return this.NearbyTransducers;
    }
}
