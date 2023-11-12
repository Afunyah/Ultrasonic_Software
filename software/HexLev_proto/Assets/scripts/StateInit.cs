using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class StateInit : MonoBehaviour
{

    private Transform[] BottArr;
    private Transform[] TopArr;

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
        BottArr = gameObject.transform.Find("BottomPlate/BottomTransducers").GetComponentsInChildren<Transform>();
        TopArr = gameObject.transform.Find("TopPlate/TopTransducers").GetComponentsInChildren<Transform>();

        Transform[][] tArrs = { BottArr, TopArr };

        int i = 0;
        foreach (Transform[] arr in tArrs)
        {
            int j = 0;
            // Add the transducer class to keep transducer properties separate
            foreach (Transform child in arr)
            {
                Transducer tr = child.AddComponent<Transducer>();
                tr.Init(i, j);
                j++;
            }
            i++;
        }


    }
}
