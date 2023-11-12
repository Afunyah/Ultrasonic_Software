using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Transducer : MonoBehaviour
{
    private Vector3 tPosition;
    private int tIndex;
    private int tArr;
    private int tPhase;
    private int tAmplitude;

    void Start()
    {
    }

    void Update()
    {
    }

    public void Init(int arr, int ind)
    {
        tPhase = 0;
        tAmplitude = 0;
        tPosition = this.transform.position;
        tArr = arr;
        tIndex = ind;
        float bCenter = tArr == 0 ? 0.008F : -0.005F;

        BoxCollider Bcollider = this.AddComponent<BoxCollider>();
        Bcollider.center = new Vector3(0, 0, bCenter);
        Bcollider.size = new Vector3(0.006858086F, 0.006858086F, 0);
    }

}
