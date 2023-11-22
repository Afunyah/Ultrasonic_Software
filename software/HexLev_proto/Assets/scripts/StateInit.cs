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

    public void CalculateStateChange()
    {
        List<List<List<(Transducer, int, int)>>> CombinedPATList = new List<List<List<(Transducer, int, int)>>> { };

        List<LevParticle> particles;
        particles = GameObject.FindObjectsByType<LevParticle>(FindObjectsSortMode.None).ToList();

        List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> CombinedFTTDList;
        CombinedFTTDList = new List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> { };
        foreach (LevParticle particle in particles)
        {
            CombinedFTTDList.Add(particle.GetFullTrajectoryTransducerDataList());
        }

        List<List<(Transducer, int, int)>> ParticlePATList = new List<List<(Transducer, int, int)>> { };
        foreach (List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> FTTDList in CombinedFTTDList)
        {
            List<(Transducer, int, int)> PAList = new List<(Transducer, int, int)> { };
            foreach ((List<GhostTransducerPositionData>, List<GhostTransducerPositionData>) GTPD_tup in FTTDList)
            {
                foreach (GhostTransducerPositionData GTPD1 in GTPD_tup.Item1)
                {
                    PAList.Add(ConvertGTPDtoPhaseAmplitude(GTPD1, false));
                }

                foreach (GhostTransducerPositionData GTPD2 in GTPD_tup.Item2)
                {
                    PAList.Add(ConvertGTPDtoPhaseAmplitude(GTPD2, true));
                }
                ParticlePATList.Add(PAList);
            }

            CombinedPATList.Add(ParticlePATList);
        }



        foreach (List<List<(Transducer, int, int)>> PartiPatList in CombinedPATList)
        {
            Debug.Log("Particle X: ");
            foreach (List<(Transducer, int, int)> item in PartiPatList)
            {
                foreach ((Transducer, int, int) dat in item)
                {
                    Debug.Log(dat.Item1 + " {Phase: " + dat.Item2 + " Amplitude: " + dat.Item3 + " }");
                }
            }
        }
    }

    // public bool Update

    public (Transducer, int, int) ConvertGTPDtoPhaseAmplitude(GhostTransducerPositionData gtpd, bool far)
    {
        int phase = 0;
        int amplitude = 0;
        float dist = gtpd.GetDist();
        float ang = gtpd.ang;
        Transducer trs = gtpd.trs;

        // Very crude implementation that only caters to specific snaps.
        // Make ranges linear to give more snaps

        // {C->T:C}, {T->C:C}, {C->B:C,B}, {B->C:C} 
        if (dist <= 0.075f && ((ang >= 175 && ang <= 185) || (ang >= 0 && ang <= 10)))
        {
            phase = 100;
            amplitude = 100;
        }

        // {C->T:T}
        else if (dist <= 0.075f && (ang >= 55 && ang <= 65))
        {
            phase = 100;
            amplitude = 100;
        }

        // {T->C:T}, {T->B:T}, {B->C:B}
        else if ((dist >= 0.075f && dist <= 0.13f) && ((ang >= 145 && ang <= 155) || (ang >= 175 && ang <= 185)))
        {
            phase = 0;
            amplitude = 100;
        }

        // {T->B:B}
        else if (dist <= 0.075f && (ang >= 85 && ang <= 95))
        {
            phase = 100;
            amplitude = 100;
        }

        // {B->T:T,B}
        else if ((dist >= 0.075f && dist <= 0.13f) && ((ang >= 0 && ang <= 10) || (ang >= 115 && ang <= 125)))
        {
            phase = 100;
            amplitude = 100;
        }

        if (far)
        {
            phase = 0;
            amplitude = 100;
        }


        return (trs, phase, amplitude);
    }
}
