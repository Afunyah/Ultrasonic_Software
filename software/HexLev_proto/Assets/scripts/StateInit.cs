using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class StateInit : MonoBehaviour
{

    private List<Transducer> BottArray;
    private List<Transducer> TopArray;
    private float HexCntr_z;

    void Awake()
    {
        BottArray = new List<Transducer> { };
        TopArray = new List<Transducer> { };
    }
    void Start()
    {
        InitializeArrays();
    }

    // Update is called once per frame
    void Update()
    {

    }

    public void InitializeArrays()
    {
        Transform[] BArr;
        Transform[] TArr;
        BArr = this.gameObject.transform.Find("BottomPlate/BottomTransducers").GetComponentsInChildren<Transform>();
        TArr = this.gameObject.transform.Find("TopPlate/TopTransducers").GetComponentsInChildren<Transform>();
        HexCntr_z = this.gameObject.transform.Find("HexCenter").GetComponent<Transform>().transform.localPosition.y;
        Transform[][] tArrs = { BArr, TArr };

        int i = 0;
        foreach (Transform[] arr in tArrs)
        {
            int j = 0;
            foreach (Transform child in arr)
            {
                if (j == 0) { j++; continue; }
                Transducer tr = child.AddComponent<Transducer>();
                tr.Init(i, j - 1, HexCntr_z);
                if (i == 0) { BottArray.Add(tr); }
                if (i == 1) { TopArray.Add(tr); }
                j++;
            }
            i++;
        }

    }

    public void UpdateLevState()
    {
        List<LevParticle> particles;
        particles = GameObject.FindObjectsByType<LevParticle>(FindObjectsSortMode.None).ToList();

        List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> FTTDList;
        foreach (LevParticle particle in particles)
        {
            FTTDList = particle.GetFullTrajectoryTransducerDataList();
        }
        
        
    }
}
