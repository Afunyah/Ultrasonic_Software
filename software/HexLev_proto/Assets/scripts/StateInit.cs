using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class StateInit : MonoBehaviour
{

    private Transform[] BottArr;
    private Transform[] TopArr;
    private float HexCntr_z;

    void Start()
    {
        InitializeArrays();
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void InitializeArrays()
    {
        BottArr = this.gameObject.transform.Find("BottomPlate/BottomTransducers").GetComponentsInChildren<Transform>();
        TopArr = this.gameObject.transform.Find("TopPlate/TopTransducers").GetComponentsInChildren<Transform>();
        HexCntr_z = this.gameObject.transform.Find("HexCenter").GetComponent<Transform>().transform.localPosition.y;
        // Debug.Log(HexCntr_z);
        Transform[][] tArrs = { BottArr, TopArr };
        // Debug.Log(BottArr[0]);

        int i = 0;
        foreach (Transform[] arr in tArrs)
        {
            int j = 0;
            // Add the transducer class to keep transducer properties separate
            foreach (Transform child in arr)
            {
                if (j == 0) { j++; continue; }
                Transducer tr = child.AddComponent<Transducer>();
                tr.Init(i, j-1, HexCntr_z);
                j++;
            }
            i++;
        }


    }
}
