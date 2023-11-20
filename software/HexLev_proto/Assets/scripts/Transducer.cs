using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

public class Transducer : MonoBehaviour
{
    private Vector3 tPosition;
    private int tIndex;
    private int tArr;
    private int tPhase;
    private int tAmplitude;

    private bool used;

    void Start()
    {
    }

    void Update()
    {
    }

    public void Init(int arr, int ind, float bCenter_z)
    {
        used = false;
        tPhase = 0;
        tAmplitude = 0;
        tPosition = this.transform.position;
        tArr = arr;
        tIndex = ind;
        float bOff = tArr == 0 ? 1F : -1F;
        if (arr == 1 && ind == 1)
        {
        }

        BoxCollider Bcollider = this.AddComponent<BoxCollider>();
        // Bcollider.center = new Vector3(0, 0, (bCenter_z-this.gameObject.transform.position.y)/10-this.gameObject.transform.localPosition.z-bOff*((bCenter_z-this.gameObject.transform.position.y)/10));

        // Bcollider.center = new Vector3(0, 0, ((bCenter_z - this.gameObject.transform.position.y) / 10 - (this.gameObject.transform.localPosition.z)) + 0);
        Bcollider.center = new Vector3(0, 0, ((bCenter_z ) / 14.5813F )*bOff - bOff*0.00201F);

        Bcollider.size = new Vector3(0.006858086F, 0.006858086F, ((bCenter_z ) / 10)*2 - 2*0.04F - 2*0.00201F);
        this.gameObject.layer = LayerMask.NameToLayer("TransducerLayer");
    }

    public void Activate(){
        used = true;
    }

    public void Deactivate(){
        used = false;
    }

    public bool IsActive(){
        return used;
    }

    public Vector3 GetPosition(){
        return tPosition;
    }

    public Vector2 GetXYPosition(){
        return new Vector2(Mathf.Round(this.GetPosition().x*100)/100, Mathf.Round(this.GetPosition().z*100)/100);
    }

    public int GetPhase(){
        return this.tPhase;
    }

    public int GetAmplitude(){
        return this.tAmplitude;
    }

    public void SetPhase(int p){
        this.tPhase = p;
    }

    public void SetAmplitude(int a){
        this.tAmplitude = a;
    }

}
