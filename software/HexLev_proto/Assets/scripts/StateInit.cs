using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO.Ports;
using Unity.VisualScripting;
using UnityEngine;
using System.Threading.Tasks;
using System;
using System.Numerics;
using UnityEditor;
using System.Text.RegularExpressions;

using Vector3 = UnityEngine.Vector3;
using UnityEngine.AI;
using Accord.Math;
using UnityEngine.Assertions;

/// <summary>
/// Initialises the program state: handles external interfacing and control algorithms
/// </summary>
public class StateInit : MonoBehaviour
{


    /// <summary>
    /// Serial Port for arduino. Will differ based on ports and OS
    /// </summary>
    // private List<SerialPort> FPGA_Serial;
    private SerialPort[] FPGA_Serial;

    /// <summary>
    /// Array of transducers for bottom plate
    /// </summary>
    // private List<Transducer> BottArray;

    // /// <summary>
    // /// Array of transducers for top plate
    // /// </summary>
    // private List<Transducer> TopArray;

    private double[] hexCoords;

    public Transducer[] BottArray;

    /// <summary>
    /// Array of transducers for top plate
    /// </summary>
    public Transducer[] TopArray;


    public List<Transducer> BottArray_tmp;

    /// <summary>
    /// Array of transducers for top plate
    /// </summary>
    public List<Transducer> TopArray_tmp;

    /// <summary>
    /// Z-axis value of the levitator centre.
    /// </summary>
    private float HexCntr_z;

    // private TextAsset offset_bottom_csv;
    // private TextAsset offset_top_csv;

    // private string[] offset_bottom_text;
    // private string[] offset_top_text;
    // private double[] offset_bottom;
    // private double[] offset_top;

    private double[] offsets;

    private int[] unityIndexArray;
    private int[] fpgaIndexArray;
    private int[] inBankIndexArray;
    void Awake()
    {
        // StartSerial();

    }
    void Start()
    {
        fpgaIndexArray = Labeller.GetFpgaIndexArray();
        inBankIndexArray = Labeller.GetInBankIndexArray();
        unityIndexArray = Labeller.GetUnityIndexArray();

        offsets = Labeller.GetOffsets();

        InitializeArrays();
    }

    // Update is called once per frame
    void Update()
    {

    }


    private void StartSerial()
    {

        // FPGA_Serial.AddRange(new List<SerialPort>() { new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort()});
        bool fpgas_found = true;
        FPGA_Serial = new SerialPort[6];
        int[] FPGA_ids = {-1,-1,-1,-1,-1,-1};
        string[] ports = SerialPort.GetPortNames();

        foreach (string port in ports)
        {
            if (!port.Contains("tty.usbserial"))
            {
                continue;
            }

            // Debug.Log(port);

            try
            {
                SerialPort ser = new SerialPort(port, 115200);

                while (!ser.IsOpen)
                {
                    ser.ReadTimeout = 3000;
                    ser.WriteTimeout = 1500;
                    ser.DtrEnable = true;
                    ser.RtsEnable = true;
                    ser.Handshake = Handshake.None;
                    ser.Open();
                }

                // ser.ReadTimeout = 1000;

                byte[] fnd = StringToByteArray("AB");
                // string ack = "";
                ser.Write(fnd, 0, 1);
                // ser.BaseStream.Flush(); 
                int ack = ser.ReadByte();
                // while (ack != serial_command)
                // {
                //     ack = ser.ReadLine();
                // }

                // Debug.Log(ack);

                if (ack < 6)
                {
                    FPGA_Serial[ack] = new SerialPort(port, 115200);
                    while (!FPGA_Serial[ack].IsOpen)
                    {
                        FPGA_Serial[ack].ReadTimeout = 2000;
                        FPGA_Serial[ack].WriteTimeout = 1500;
                        FPGA_Serial[ack].DtrEnable = true;
                        FPGA_Serial[ack].RtsEnable = true;
                        FPGA_Serial[ack].Handshake = Handshake.None;
                        FPGA_Serial[ack].Open();
                    }
                    // Debug.Log("BEFORE");
                    // Debug.Log(GetPixel(ser, 0));
                    // Debug.Log("HERE");
                    // Debug.Log(FPGA_Serial.Count);
                    FPGA_ids[ack] = ack;
                }
                else
                {
                    ser.Close();
                }
            }
            // catch (Exception ex)
            // {
            //     // Debug.Log(ex);
            // }
            finally{
                
            }

        }

        for (int i = 0; i < FPGA_ids.Count(); i++)
        {
            if(FPGA_ids[i] != i){
                fpgas_found = false;
                Debug.Log("Not found, FPGA: " + i);
            }
        }

    if (!fpgas_found){
            Debug.Log("FPGAS HAVE NOT BEEN FOUND");
            throw new Exception("FPGAS HAVE NOT BEEN FOUND");
    }

    }

    /// <summary>
    /// Adds transducers to their respective plate arrays. The transducers are pre-labelled in Unity Editor.
    /// </summary>
    public void InitializeArrays()
    {
        // labeller = this.gameObject.GetComponent<Labeller>();

        hexCoords = Solver.GetHexCoords();



        // BottArray = new List<Transducer>(721);
        // TopArray = new List<Transducer>(721);
        BottArray = new Transducer[721];
        TopArray = new Transducer[721];

        BottArray_tmp = new List<Transducer>();
        TopArray_tmp = new List<Transducer>();


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
                // try
                // {
                if (j == 0) { j++; continue; }
                Transducer tr = child.AddComponent<Transducer>();
                string obnm = child.gameObject.name;
                int bid = Int32.Parse(Regex.Match(obnm, @"\d+").Value);
                int s_ind = unityIndexArray.IndexOf(bid);

                tr.Init(i, j - 1, HexCntr_z, bid);

                tr.SetSolverIndex(s_ind + 1);
                tr.SetFpgaIndex(fpgaIndexArray[s_ind]);
                tr.SetInBankIndex(inBankIndexArray[s_ind]);
                

                if (i == 0) { BottArray[s_ind] = tr; tr.SetSolverPostion(new Vector3((float)hexCoords[s_ind], 0, (float)hexCoords[s_ind+721]));}
                if (i == 1) { TopArray[s_ind] = tr; tr.SetSolverPostion(new Vector3((float)hexCoords[s_ind], (float)Solver.GetTdist(), (float)hexCoords[s_ind+721]));}
                j++;

                // }
                // catch (Exception ex)
                // {
                //     Debug.Log(ex);
                // }
            }
            i++;
        }

        for (int k = 0; k < 1; k++)
        {
            BottArray_tmp.Add(BottArray[k]);
            TopArray_tmp.Add(TopArray[k]);
            // Debug.Log(TopArray[k].GetSolverPostion());
        }

        // foreach (Transducer item in BottArray)
        // {
        //     Debug.Log(item.GetUnityIndex() + " " + item.GetSolverIndex() + " " + item.GetFpgaIndex() + " " + item.GetInBankIndex());
        // }

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
    // public bool UpdateLevState()
    // {
    //     // All moves performed?
    //     bool exhausted = false;

    //     List<List<List<(Transducer, int, int)>>> CombinedPATList = this.CalculateStateChange();

    //     // Executed while moves are available
    //     while (!exhausted)
    //     {
    //         exhausted = true; // False Flag if moves are still available

    //         // The action list contains the extracted phases and amplitudes per transducer in a single timestep.
    //         // Essentialy a queue
    //         List<(Transducer, int, int)> ActionList = new List<(Transducer, int, int)> { };
    //         foreach (List<List<(Transducer, int, int)>> ParticlePATList in CombinedPATList)
    //         {
    //             if (ParticlePATList.Count != 0)
    //             {
    //                 ActionList.AddRange(ParticlePATList[0]);
    //                 ParticlePATList.RemoveAt(0);
    //                 exhausted = false;
    //             }
    //         }

    //         // Set phases and amplitudes through the arduino
    //         foreach ((Transducer, int, int) PATrsData in ActionList)
    //         {
    //             Transducer trs = PATrsData.Item1;
    //             int phase = PATrsData.Item2;
    //             int amplitude = PATrsData.Item3;
    //             trs.SetPhase(phase);
    //             trs.SetAmplitude(amplitude);

    //             // DEBUG VALUES
    //             int fpga_addr = 0;
    //             int bank_num = 0;
    //             string pixel_num = "";
    //             int config_data = 0;

    //             switch (trs.name)
    //             {
    //                 case "Transducer.189":
    //                     pixel_num = "0";
    //                     break;
    //                 case "Transducer.101":
    //                     pixel_num = "1";
    //                     break;
    //                 case "Transducer.304":
    //                     pixel_num = "2";
    //                     break;
    //                 case "Transducer.214":
    //                     pixel_num = "3";
    //                     break;
    //                 case "Transducer.184":
    //                     pixel_num = "4";
    //                     break;
    //                 case "Transducer.290":
    //                     pixel_num = "5";
    //                     break;
    //                 case "Transducer.291":
    //                     pixel_num = "6";
    //                     break;
    //                 default:
    //                     continue;
    //             }

    //             int b_amp = 0;
    //             int b_phs = 0;
    //             if (amplitude == 100)
    //             {
    //                 b_amp = 128;
    //             }
    //             if (phase == 100)
    //             {
    //                 b_amp = 8;
    //             }
    //             config_data = b_amp + b_phs;

    //             string serial_command = string.Format("SET_PIXEL {0} {1} {2} {3}", fpga_addr, bank_num, pixel_num, config_data);
    //             Debug.Log("SENT: " + serial_command);
    //             try
    //             {
    //                 // Task t = Task.Run(() =>
    //                 // {

    //                 // });

    //                 // if (!t.Wait(10000))
    //                 // {
    //                 //     throw new System.Exception("Serial Timeout");
    //                 // }

    //                 string ack = "";
    //                 ArduinoSerial.WriteLine(serial_command);
    //                 ArduinoSerial.BaseStream.Flush();
    //                 ack = ArduinoSerial.ReadLine();
    //                 while (ack != serial_command)
    //                 {
    //                     ack = ArduinoSerial.ReadLine();
    //                 }
    //                 Debug.Log(ack);
    //             }
    //             catch (System.Exception e)
    //             {
    //                 Debug.LogException(e);
    //                 return false;
    //             }
    //             // Debug.Log("Here");
    //         }
    //     }

    //     return true;
    // }

    // public bool UpdateLevState()
    // {
    //     List<LevParticle> particles;
    //     particles = GameObject.FindObjectsByType<LevParticle>(FindObjectsSortMode.None).ToList();

    //     LevParticle parti = particles[0];


    //     // foreach (SerialPort item in FPGA_Serial)
    //     // {
    //     //     try
    //     //     {
    //     //         SetPixel(item, 0, 5, 1);
    //     //         item.BaseStream.Flush();
    //     //         Debug.Log(GetPixel(item, 0));
    //     //     }
    //     //     catch (System.Exception e)
    //     //     {

    //     //         Debug.Log(e);
    //     //     }
    //     // }

    //     return true;
    // }

    // public bool UpdateLevState()
    // {
    //     // All moves performed?
    //     bool exhausted = false;

    //     List<List<List<(Transducer, int, int)>>> CombinedPATList = this.CalculateStateChange();

    //     // Executed while moves are available
    //     while (!exhausted)
    //     {
    //         exhausted = true; // False Flag if moves are still available

    //         // The action list contains the extracted phases and amplitudes per transducer in a single timestep.
    //         // Essentialy a queue
    //         List<(Transducer, int, int)> ActionList = new List<(Transducer, int, int)> { };
    //         foreach (List<List<(Transducer, int, int)>> ParticlePATList in CombinedPATList)
    //         {
    //             if (ParticlePATList.Count != 0)
    //             {
    //                 ActionList.AddRange(ParticlePATList[0]);
    //                 ParticlePATList.RemoveAt(0);
    //                 exhausted = false;
    //             }
    //         }

    //         // Set phases and amplitudes through the arduino
    //         foreach ((Transducer, int, int) PATrsData in ActionList)
    //         {
    //             Transducer trs = PATrsData.Item1;
    //             int phase = PATrsData.Item2;
    //             int amplitude = PATrsData.Item3;
    //             trs.SetPhase(phase);
    //             trs.SetAmplitude(amplitude);

    //             // DEBUG VALUES
    //             int fpga_addr = 0;
    //             int bank_num = 0;
    //             string pixel_num = "";
    //             int config_data = 0;

    //             string serial_command = string.Format("SET_PIXEL {0} {1} {2} {3}", fpga_addr, bank_num, pixel_num, config_data);

    //             try
    //             {
    //                 // Task t = Task.Run(() =>
    //                 // {

    //                 // });

    //                 // if (!t.Wait(10000))
    //                 // {
    //                 //     throw new System.Exception("Serial Timeout");
    //                 // }
    //             }
    //             catch (System.Exception e)
    //             {
    //                 Debug.LogException(e);
    //                 return false;
    //             }
    //             // Debug.Log("Here");
    //         }
    //     }

    //     return true;
    // }


    public void UpdateLevState(){
        var temptime = Time.realtimeSinceStartup;
        (double[], List<bool>) sol = this.GetComponent<Solver>().Solve();
        double[] phis = sol.Item1;
        List<bool> activePhis = sol.Item2;
        
        for (int i = 0; i < 1442; i++)
        {
            phis[i] = phis[i] - offsets[i];

            phis[i] = phis[i] % 2 * Math.PI;
            phis[i] = phis[i] * 31/(2*Math.PI);
            phis[i] = Math.Round(phis[i]); 
        }

        for (int i = 0; i < 721; i++)
        {
            int bott_ind = i;
            int top_ind = i + 721;

            // SetPixel(FPGA_Serial[fpgaIndexArray[bott_ind]], inBankIndexArray[bott_ind], 4, (int) phis[bott_ind]);
            // SetPixel(FPGA_Serial[fpgaIndexArray[top_ind]], inBankIndexArray[top_ind], 4, (int) phis[top_ind]);
        }

        Debug.Log ("Time taken for solver and serial output : "+(Time.realtimeSinceStartup - temptime));

        
        // for (int i = 0; i < 241; i++)
        // {
        //     foreach (SerialPort fpga_ser in FPGA_Serial)
        //     {
        //         SetPixel(fpga_ser, i, 4, 2);
        //     }
        //     Debug.Log("Set "+i);
        // }
        // SetPixel(FPGA_Serial[0], 1, 4, 2);
        // Debug.Log(GetPixel(FPGA_Serial[0], 1));
    }

    private void SetPixel(SerialPort ser, int address, int duty, int phase)
    {
        string set_cmd = string.Format("CD{0:X4}{1:X2}{2:X2}", address, duty, phase);
        byte[] byteArray = StringToByteArray(set_cmd);
        ser.Write(byteArray, 0, 5);
    }

    private (int,int) GetPixel(SerialPort ser, int address)
    {
        // Debug.Log("Attempting to get Pixel");
        string get_cmd = string.Format("EF{0:X4}", address);
        byte[] byteArray = StringToByteArray(get_cmd);
        ser.Write(byteArray, 0, 3);
        int ack0 = ser.ReadByte();
        int ack1 = ser.ReadByte();
        return (ack0,ack1);


        // get_cmd = "EF{:04X}".format(address)
        // ser.write(bytes.fromhex(get_cmd))
        // val = ser.read(2)
        // return (int(val[0]), int(val[1]))
    }


    public static byte[] StringToByteArray(string hex)
    {
        return Enumerable.Range(0, hex.Length)
                         .Where(x => x % 2 == 0)
                         .Select(x => Convert.ToByte(hex.Substring(x, 2), 16))
                         .ToArray();
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
        foreach (SerialPort fpga in FPGA_Serial)
        {
            try
            {

                fpga.Close();

            }
            catch (Exception ex)
            {
                Debug.Log(ex);
            }
        }
    }
}
