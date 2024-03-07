using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO.Ports;
using Unity.VisualScripting;
using UnityEngine;
using System.Threading.Tasks;
using System;

/// <summary>
/// Initialises the program state: handles external interfacing and control algorithms
/// </summary>
public class StateInit : MonoBehaviour
{

    /// <summary>
    /// Serial Port for arduino. Will differ based on ports and OS
    /// </summary>
    private SerialPort ArduinoSerial;

    /// <summary>
    /// Array of transducers for bottom plate
    /// </summary>
    private List<Transducer> BottArray;

    /// <summary>
    /// Array of transducers for top plate
    /// </summary>
    private List<Transducer> TopArray;

    /// <summary>
    /// Z-axis value of the levitator centre.
    /// </summary>
    private float HexCntr_z;

    void Awake()
    {
        // Change arduino port and baud rate to match hardware
        ArduinoSerial = new SerialPort("/dev/tty.usbmodem1101", 9600);
        BottArray = new List<Transducer> { };
        TopArray = new List<Transducer> { };

        while (!ArduinoSerial.IsOpen)
        {
            ArduinoSerial.ReadTimeout = 3000;
            ArduinoSerial.WriteTimeout = 1000;
            ArduinoSerial.DtrEnable = true;
            ArduinoSerial.RtsEnable = true;
            ArduinoSerial.Handshake = Handshake.None;
            ArduinoSerial.Open();
        }
    }
    void Start()
    {
        InitializeArrays();
    }

    // Update is called once per frame
    void Update()
    {

    }

/// <summary>
/// Adds transducers to their respective plate arrays. The transducers are pre-labelled in Unity Editor.
/// </summary>
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

    /// <summary>
    /// Calculates the state change for needed to advance all particles per timestep.
    /// The combined PAT (Phase-Array-Time) list is returned. 
    /// The CombinedPATList represents a combination of the Phase and Amplidute formats of each trajectory for each particle
    /// The 'Time' aspect is introduced when each movement of each particle is aligned. 
    /// This indicates that Move 1 of every particle occurs in the same step, then Move 2 of every particle occurs next...
    /// </summary>
    /// <returns>A list of lists containing lists of 3-item tuples. (Transducer, Phase, Amplitude)</returns>
    private List<List<List<(Transducer, int, int)>>> CalculateStateChange()
    {
        List<List<List<(Transducer, int, int)>>> CombinedPATList = new List<List<List<(Transducer, int, int)>>> { };

        //Find all particles
        List<LevParticle> particles;
        particles = GameObject.FindObjectsByType<LevParticle>(FindObjectsSortMode.None).ToList();

        // For each particle, call GetFullTrajectoryTransducerDataList() and combine the information of all the trajectories
        List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> CombinedFTTDList;
        CombinedFTTDList = new List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> { };
        foreach (LevParticle particle in particles)
        {
            CombinedFTTDList.Add(particle.GetFullTrajectoryTransducerDataList());
        }

        // For each individual trajectory data (or for each particle) convert the GhostTransducerPositionData into phases and amplitudes
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


        // Debugging
        // foreach (List<List<(Transducer, int, int)>> PartiPatList in CombinedPATList)
        // {
        //     Debug.Log("Particle X: ");
        //     foreach (List<(Transducer, int, int)> item in PartiPatList)
        //     {
        //         foreach ((Transducer, int, int) dat in item)
        //         {
        //             Debug.Log(dat.Item1 + " {Phase: " + dat.Item2 + " Amplitude: " + dat.Item3 + " }");
        //         }
        //     }
        // }

        return CombinedPATList;
    }

    /// <summary>
    /// Updates the state of the physical levitator.
    /// Interfaces with an arduino
    /// TODO: (Dependency) Implement updator for large array when PCB and Transducers are available
    /// Current placeholde uses debug values
    /// </summary>
    /// <returns></returns>
    public bool UpdateLevState()
    {
        // All moves performed?
        bool exhausted = false;

        List<List<List<(Transducer, int, int)>>> CombinedPATList = this.CalculateStateChange();

        // Executed while moves are available
        while (!exhausted)
        {
            exhausted = true; // False Flag if moves are still available

            // The action list contains the extracted phases and amplitudes per transducer in a single timestep.
            // Essentialy a queue
            List<(Transducer, int, int)> ActionList = new List<(Transducer, int, int)> { };
            foreach (List<List<(Transducer, int, int)>> ParticlePATList in CombinedPATList)
            {
                if (ParticlePATList.Count != 0)
                {
                    ActionList.AddRange(ParticlePATList[0]);
                    ParticlePATList.RemoveAt(0);
                    exhausted = false;
                }
            }

            // Set phases and amplitudes through the arduino
            foreach ((Transducer, int, int) PATrsData in ActionList)
            {
                Transducer trs = PATrsData.Item1;
                int phase = PATrsData.Item2;
                int amplitude = PATrsData.Item3;
                trs.SetPhase(phase);
                trs.SetAmplitude(amplitude);

                // DEBUG VALUES
                int fpga_addr = 0;
                int bank_num = 0;
                string pixel_num = "";
                int config_data = 0;
                
                switch (trs.name)
                {
                    case "Transducer.189":
                        pixel_num = "0";
                        break;
                    case "Transducer.101":
                        pixel_num = "1";
                        break;
                    case "Transducer.304":
                        pixel_num = "2";
                        break;
                    case "Transducer.214":
                        pixel_num = "3";
                        break;
                    case "Transducer.184":
                        pixel_num = "4";
                        break;
                    case "Transducer.290":
                        pixel_num = "5";
                        break;
                    case "Transducer.291":
                        pixel_num = "6";
                        break;
                    default:
                        continue;
                }

                int b_amp = 0;
                int b_phs = 0;
                if (amplitude == 100)
                {
                    b_amp = 128;
                }
                if (phase == 100)
                {
                    b_amp = 8;
                }
                config_data = b_amp + b_phs;

                string serial_command = string.Format("SET_PIXEL {0} {1} {2} {3}", fpga_addr, bank_num, pixel_num, config_data);
                Debug.Log("SENT: " + serial_command);
                try
                {
                    // Task t = Task.Run(() =>
                    // {

                    // });

                    // if (!t.Wait(10000))
                    // {
                    //     throw new System.Exception("Serial Timeout");
                    // }

                    string ack = "";
                    ArduinoSerial.WriteLine(serial_command);
                    ArduinoSerial.BaseStream.Flush();
                    ack = ArduinoSerial.ReadLine();
                    while (ack != serial_command)
                    {
                        ack = ArduinoSerial.ReadLine();
                    }
                    Debug.Log(ack);
                }
                catch (System.Exception e)
                {
                    Debug.LogException(e);
                    return false;
                }
                // Debug.Log("Here");
            }
        }

        return true;
    }
    

    /// <summary>
    /// Converts GhostTransducerPositionData to phase and amplitude.
    /// The current placeholder is a crude implementation which switches phases and amplitudes from low to high based on distances and angles.
    /// The positions availabe are Triangle (T, 3 transducers), Between (B, 2 transducers), Center (C, 1 transducer)
    /// TODO: (Dependency) Implement solver for large array when PCB and Transducers are available
    /// </summary>
    /// <param name="gtpd">GhostTransducerPositionData to extract data from</param>
    /// <param name="far">Indicates whether a transducer is within the Area-Of-Interest of a particle (currently unused)</param>
    /// <returns></returns>
    public (Transducer, int, int) ConvertGTPDtoPhaseAmplitude(GhostTransducerPositionData gtpd, bool far)
    {
        int phase = 0;
        int amplitude = 0;
        float dist = gtpd.GetDist();
        float ang = gtpd.ang;
        Transducer trs = gtpd.trs;

        // Very crude implementation that only caters to specific snaps.

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

    public void OnApplicationQuit()
    {
        ArduinoSerial.Close();
    }
}
