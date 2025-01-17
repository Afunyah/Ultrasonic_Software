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
using Vector2 = UnityEngine.Vector2;
using UnityEngine.AI;
using Accord.Math;
using UnityEngine.Assertions;
using MathNet.Numerics.Random;


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

    public bool SERIALON;

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

    public int[] unityIndexArray;
    private int[] fpgaIndexArray;
    private int[] inBankIndexArray;
    public int[] xyTestIndexArray;

    private Vector3 solvrMaxVector;
    private Vector3 solvrMinVector;
    private Vector3 solvrCenterVector;
    private Vector3 solvrSizeVector;


    bool BRD_DEBUG;
    private int BOTT_BRD;
    private int TOP_BRD;

    String T_POS;

    void Awake()
    {
        SERIALON = false;
        BRD_DEBUG = false;
        if (SERIALON)
        {
            StartSerial();
        }

    }
    void Start()
    {
        fpgaIndexArray = Labeller.GetFpgaIndexArray();
        inBankIndexArray = Labeller.GetInBankIndexArray();
        unityIndexArray = Labeller.GetUnityIndexArray();
        xyTestIndexArray = Labeller.GetXYTestIndexArray();

        offsets = Labeller.GetOffsets();

        solvrMaxVector = Vector3.zero;
        solvrMinVector = Vector3.zero;
        solvrCenterVector = Vector3.zero;
        solvrSizeVector = Vector3.zero;

        BOTT_BRD = 0;
        TOP_BRD = 3;
        T_POS = BOTT_BRD.ToString() + TOP_BRD.ToString();

        InitializeArrays();
    }

    // Update is called once per frame
    void Update()
    {

    }


    public void StartSerial()
    {
        SERIALON = true;

        // FPGA_Serial.AddRange(new List<SerialPort>() { new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort(),new SerialPort()});
        bool fpgas_found = true;
        FPGA_Serial = new SerialPort[6];
        int[] FPGA_ids = { -1, -1, -1, -1, -1, -1 };
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
                    ser.ReadTimeout = 1500;
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
            catch (Exception ex)
            {
                // Debug.Log(ex);
            }

        }

        for (int i = 0; i < FPGA_ids.Count(); i++)
        {
            if (FPGA_ids[i] != i)
            {
                fpgas_found = false;
                Debug.Log("Not found, FPGA: " + i);
            }
        }

        if (!fpgas_found)
        {
            Debug.Log("FPGAS HAVE NOT BEEN FOUND");
            SERIALON = false;
            throw new Exception("FPGAS HAVE NOT BEEN FOUND");
        }
        else
        {
            Debug.Log("FPGAS AVAILABLE; SERIAL STARTED SUCCESSFULLY");
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
                int unityId = Int32.Parse(Regex.Match(obnm, @"\d+").Value);
                int solver_ind_singular = unityIndexArray.IndexOf(unityId);
                int solver_ind = 0;


                tr.Init(i, j - 1, HexCntr_z, unityId);


                if (i == 0)
                {
                    //bottom plate
                    solver_ind = solver_ind_singular;
                    // Debug.Log(hexCoords[solver_ind_singular]);
                    // tr.SetSolverPostion(new Vector3((float)hexCoords[solver_ind_singular], 0, (float)hexCoords[solver_ind_singular + 721])); // y z swapped
                    tr.SetSolverPostion(new Vector3((float)hexCoords[solver_ind_singular], (float)hexCoords[solver_ind_singular + 721], 0));
                    tr.SetNormals(new Vector3(0, 0, 1));
                }

                if (i == 1)
                {
                    //top plate
                    solver_ind = solver_ind_singular + 721;
                    // tr.SetSolverPostion(new Vector3((float)hexCoords[solver_ind_singular], (float)Solver.GetTdist(), (float)hexCoords[solver_ind_singular + 721])); // y z swapped
                    tr.SetSolverPostion(new Vector3((float)hexCoords[solver_ind_singular], (float)hexCoords[solver_ind_singular + 721], (float)Solver.GetTdist()));
                    tr.SetNormals(new Vector3(0, 0, -1));
                }

                tr.SetSolverIndex(solver_ind + 1);
                tr.SetFpgaIndex(fpgaIndexArray[solver_ind]);
                tr.SetInBankIndex(inBankIndexArray[solver_ind]);

                int xytest_ind = xyTestIndexArray.IndexOf(tr.GetSolverIndex());
                tr.SetXYTestIndex(xytest_ind);

                double phase_offset = offsets[tr.GetXYTestIndex()];
                tr.SetPhaseOffset(phase_offset);

                solvrMinVector = Vector3.Min(tr.GetSolverPostion(), solvrMinVector);
                solvrMaxVector = Vector3.Max(tr.GetSolverPostion(), solvrMaxVector);

                // if (fpgaIndexArray[solver_ind] == 5 || fpgaIndexArray[solver_ind] == 0)
                // {
                //     solvrMinVector = Vector3.Min(tr.GetSolverPostion(), solvrMinVector);
                //     solvrMaxVector = Vector3.Max(tr.GetSolverPostion(), solvrMaxVector);
                // }

                if (i == 0) { BottArray[solver_ind_singular] = tr; }
                if (i == 1) { TopArray[solver_ind_singular] = tr; }
                j++;

                // }
                // catch (Exception ex)
                // {
                //     Debug.Log(ex);
                // }
            }
            i++;
        }

        // solvrSizeVector = solvrMaxVector - solvrMinVector;
        // // solvrCenterVector = solvrMinVector + 0.5f * Vector3.Normalize(new Vector3(solvrSizeVector.x,solvrSizeVector.y));
        // solvrCenterVector = solvrMinVector + 0.5f * solvrSizeVector;


        if (BRD_DEBUG)
        {
            solvrSizeVector = new Vector3(0.32f, 0.143f, 0.244f);
        }
        else
        {
            solvrSizeVector = new Vector3(0.32f, 0.28f, 0.244f);
        }
        // solvrSizeVector = new Vector3(0.249f, 0.286f, 0.244f);


        // solvrCenterVector = new Vector3(0f, -0.071f, 0.122f); // Top A Bott A
        // solvrCenterVector = new Vector3(0f, -0.071f, 0.122f); // Top B Bott B

        // solvrCenterVector = new Vector3(-0.062f,-0.036f, 0.122f); // Top C Bott A*
        // solvrCenterVector = new Vector3(-0.062f, 0.036f, 0.122f); // Top C Bott B

        // solvrCenterVector = new Vector3(0.062f, -0.036f, 0.122f); // Bott C Top A
        // solvrCenterVector = new Vector3(0.062f, 0.036f, 0.122f); // Bott C Top B

        solvrCenterVector = new Vector3(0f, 0f, 0.122f);
        // FindCenter();

        // switch (T_POS)
        // {
        //     case "03":
        //         solvrCenterVector = new Vector3(0f, -0.071f, 0.122f); // 03
        //         break;
        //     case "14":
        //         solvrCenterVector = new Vector3(0f, 0.071f, 0.122f); // 14
        //         break;
        //     case "05":
        //         solvrCenterVector = new Vector3(-0.062f, -0.036f, 0.122f); // 05
        //         break;
        //     case "15":
        //         solvrCenterVector = new Vector3(-0.062f, 0.036f, 0.122f); // 15
        //         break;
        //     case "23":
        //         solvrCenterVector = new Vector3(0.062f, -0.036f, 0.122f); // 23
        //         break;
        //     case "24":
        //         solvrCenterVector = new Vector3(0.062f, 0.036f, 0.122f); // 24
        //         break;
        //     default:
        //         solvrCenterVector = new Vector3(0f, 0f, 0f);
        //         break;
        // }

        // solvrCenterVector = new Vector3(0f, -0.071f, 0.122f); // 03
        // solvrCenterVector = new Vector3(0f, 0.071f, 0.122f); // 14

        // solvrCenterVector = new Vector3(-0.062f,-0.036f, 0.122f); // 05
        // solvrCenterVector = new Vector3(-0.062f, 0.036f, 0.122f); // 15

        // solvrCenterVector = new Vector3(0.062f, -0.036f, 0.122f); // 23
        // solvrCenterVector = new Vector3(0.062f, 0.036f, 0.122f); // 24

        // solvrSizeVector = 

        // Debug.Log(BottArray[691].GetSolverPostion().ToString("F4"));
        // Debug.Log(TopArray[1412-721].GetSolverPostion().ToString("F4"));
        // Debug.Log(BottArray[4].GetSolverPostion().ToString("F4"));
        // Debug.Log(solvrMinVector.ToString("F4"));
        // Debug.Log(solvrMaxVector.ToString("F4"));
        // Debug.Log(solvrSizeVector.ToString("F4"));
        // Debug.Log(solvrCenterVector.ToString("F4"));



        // for (int k = 0; k < 1; k++)
        // {
        //     BottArray_tmp.Add(BottArray[k]);
        //     TopArray_tmp.Add(TopArray[k]);
        //     // Debug.Log(TopArray[k].GetSolverPostion());
        // }

        // foreach (Transducer item in BottArray)
        // {
        //     Debug.Log(item.GetUnityIndex() + " " + item.GetSolverIndex() + " " + item.GetFpgaIndex() + " " + item.GetInBankIndex());
        // }

        for (int k = 0; k < fpgaIndexArray.Length; k++)
        {
            // Debug.Log(k+1 + " : " + fpgaIndexArray[k] + " : " + inBankIndexArray[k]);
        }

    }

    private void FindCenter()
    {


        if (BRD_DEBUG)
        {
            T_POS = BOTT_BRD.ToString() + TOP_BRD.ToString();
        }
        else
        {
            T_POS = "";
        }
        // Debug.Log(T_POS);
        switch (T_POS)
        {
            case "03":
                solvrCenterVector = new Vector3(0f, -0.071f, 0.122f); // 03
                break;
            case "14":
                solvrCenterVector = new Vector3(0f, 0.071f, 0.122f); // 14
                break;
            case "05":
                solvrCenterVector = new Vector3(-0.062f, -0.036f, 0.122f); // 05
                break;
            case "15":
                solvrCenterVector = new Vector3(-0.062f, 0.036f, 0.122f); // 15
                break;
            case "23":
                solvrCenterVector = new Vector3(0.062f, -0.036f, 0.122f); // 23
                break;
            case "24":
                solvrCenterVector = new Vector3(0.062f, 0.036f, 0.122f); // 24
                break;
            default:
                solvrCenterVector = new Vector3(0f, 0f, 0.122f);
                break;
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


    // IEnumerator SPIXEL(){
    //         SetPixel
    //         yield return new WaitForSeconds(4);
    // }

    public void D1TestFunc(int val, int on)
    {
        Debug.Log("<color=green>D1 Set</color>" + " : " + val);

        // int v = val - 1;

        // if (SERIALON)
        // {
        //     if (v > -1)
        //     {
        //         if (on == 1)
        //         {
        //             (int, int) resp = SetPixel(FPGA_Serial[fpgaIndexArray[v]], inBankIndexArray[v], 4, 0);
        //             Debug.Log("Turning ON: " + "Solver (" + val + ")" + "  FPGA (" + fpgaIndexArray[v] + ", " + inBankIndexArray[v] + ")\tResponse: " + resp);
        //         }
        //         else
        //         {
        //             (int, int) resp = SetPixel(FPGA_Serial[fpgaIndexArray[v]], inBankIndexArray[v], 0, 0);
        //             Debug.Log("Turning OFF: " + "Solver (" + val + ")" + "  FPGA (" + fpgaIndexArray[v] + ", " + inBankIndexArray[v] + ")\tResponse: " + resp);
        //         }
        //     }
        // }

        int someFpga = 0;
        if (SERIALON)
        {
            // for (int i = 0; i < 240; i++)
            // {
            //     SetPixel(FPGA_Serial[0], i, 4, 0);
            // }

            for (int i = 0; i < 240; i++)
            {
                SetPixel(FPGA_Serial[3], i, 4, 0);
            }
        }
    }

    public void D2TestFunc(int val, int on)
    {
        Debug.Log("<color=green>D2 Set</color>" + " : " + val);

        int v = val - 1;

        if (SERIALON)
        {
            if (v > -1)
            {
                if (on == 1)
                {
                    (int, int) resp = SetPixel(FPGA_Serial[fpgaIndexArray[v]], inBankIndexArray[v], 4, 0);
                    Debug.Log("Turning ON: " + "Solver (" + val + ")" + "  FPGA (" + fpgaIndexArray[v] + ", " + inBankIndexArray[v] + ")\tResponse: " + resp);
                }
                else
                {
                    (int, int) resp = SetPixel(FPGA_Serial[fpgaIndexArray[v]], inBankIndexArray[v], 0, 0);
                    Debug.Log("Turning OFF: " + "Solver (" + val + ")" + "  FPGA (" + fpgaIndexArray[v] + ", " + inBankIndexArray[v] + ")\tResponse: " + resp);
                }
            }
        }
    }


    // public void D3TestFunc(int val_1, int val_2, int on)
    // {
    //     Debug.Log("<color=green>D3 Set</color>" + " : (" + val_1 + ", " + val_2 + ")");

    //     int fpga = val_1;
    //     int bank = val_2;

    //     if (SERIALON)
    //     {
    //         if (on == 1)
    //         {
    //             (int, int) resp = SetPixel(FPGA_Serial[fpga], bank, 4, 0);
    //             Debug.Log("Turning ON: " + "FPGA (" + fpga + ", " + bank + ")\tResponse: " + resp);
    //         }
    //         else
    //         {
    //             (int, int) resp = SetPixel(FPGA_Serial[fpga], bank, 0, 0);
    //             Debug.Log("Turning OFF: " + "FPGA (" + fpga + ", " + bank + ")\tResponse: " + resp);
    //         }
    //     }
    // }

    public void D3TestFunc(int val_1, int val_2, int on)
    {
        Debug.Log("<color=green>D3 Set</color>" + " : (" + val_1 + ", " + val_2 + ")");

        BOTT_BRD = val_1;
        TOP_BRD = val_2;

        FindCenter();

        if (SERIALON)
        {
            if (on == 1)
            {
                Debug.Log("Turning ON: " + BOTT_BRD + ", " + TOP_BRD);
                UpdateLevState();
            }
            else
            {
                Debug.Log("Turning OFF ALL");

                for (int i = 0; i < 721; i++)
                {
                    int bott_ind = i;
                    int top_ind = i + 721;

                    var retrytime1 = Time.realtimeSinceStartup;
                    (int, int) resp1 = SetPixel(FPGA_Serial[fpgaIndexArray[bott_ind]], inBankIndexArray[bott_ind], 0, 0);

                    while (resp1 != (0, 0))
                    {
                        while (Time.realtimeSinceStartup - retrytime1 < 1) { }
                        resp1 = SetPixel(FPGA_Serial[fpgaIndexArray[bott_ind]], inBankIndexArray[bott_ind], 0, 0);
                    }


                    var retrytime2 = Time.realtimeSinceStartup;
                    (int, int) resp2 = SetPixel(FPGA_Serial[fpgaIndexArray[top_ind]], inBankIndexArray[top_ind], 0, 0);

                    while (resp2 != (0, 0))
                    {
                        while (Time.realtimeSinceStartup - retrytime2 < 1) { }
                        resp2 = SetPixel(FPGA_Serial[fpgaIndexArray[top_ind]], inBankIndexArray[top_ind], 0, 0);
                    }
                }
            }
        }

    }


    public void UpdateLevState()
    {
        var temptime = Time.realtimeSinceStartup;

        int phase_div = 32;
        double[] phis;
        int[] onOff;



        int opt = 3;

        if (opt == 1)
        {
            (double[], List<bool>) solvr = this.GetComponent<Solver>().Solve();

            phis = solvr.Item1;
            List<bool> activePhis = solvr.Item2;
            onOff = new int[activePhis.Count];


            for (int i = 0; i < 1442; i++)
            {
                // // Debug.Log(phis[i]);
                // // phis[i] = (phis[i]+(2 * Math.PI)) % (2 * Math.PI);
                // // phis[i] = ((phis[i] % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
                // // // phis[i] = phis[i] % (2 * Math.PI);

                // // phis[i] = phis[i] - (((offsets[i] % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI));
                // // // phis[i] = phis[i] - ((offsets[i] + (2 * Math.PI)) % (2 * Math.PI));

                // // // phis[i] = phis[i] % (2 * Math.PI);

                // // phis[i] = phis[i] * phase_div / (2 * Math.PI);
                // ;

                phis[i] = Math.Round((((phis[i] / Math.PI) % 2.0f) + ((-offsets[i]) / Math.PI)) * 16) % 32;

                // phis[i] = Math.Round(phis[i]);
                phis[i] = phis[i] >= 0 ? phis[i] : phase_div + phis[i];
                // Debug.Log(phis[i]);

                onOff[i] = activePhis[i] ? 1 : 0;
            }
        }
        else if (opt == 2)
        {
            List<Transducer> TArray = BottArray.ToList<Transducer>();
            TArray.AddRange(TopArray.ToList<Transducer>());

            Vector3 P_control_loc = new Vector3(0, 0.12f, 0.1f);

            Vector3[] emittorPosArray = new Vector3[TArray.Count];
            Vector3[] emittorNormArray = new Vector3[TArray.Count];
            Vector3[] pointArray = { P_control_loc };

            for (int i = 0; i < TArray.Count; i++)
            {
                Transducer tr = TArray[i];
                Vector3 trSolvPos = tr.GetSolverPostion();
                Vector3 trSolvNorms = tr.GetNormals();
                // Debug.Log(P_control_loc.ToString());

                emittorPosArray[i] = new Vector3(trSolvPos.x, trSolvPos.z, trSolvPos.y);
                emittorNormArray[i] = new Vector3(trSolvNorms.x, trSolvNorms.z, trSolvNorms.y);
            }


            Focus focus = this.GetComponent<Focus>();
            focus.IBP_initEmitters();
            phis = focus.FocusArrayAtPoints(emittorPosArray, emittorNormArray, pointArray);
            onOff = new int[1442];

            for (int i = 0; i < 1442; i++)
            {
                // Debug.Log(phis[i]);
                phis[i] = Math.Round(phis[i] / (2 * Math.PI) * phase_div);

                // phis[i] = phis[i] * phase_div / (2 * Math.PI);
                // phis[i] = Math.Round(phis[i]);
                // phis[i] = phis[i] >= 0 ? phis[i] : phase_div + phis[i];
                // Debug.Log(phis[i]);

                onOff[i] = 1;
            }

            // for (int i = 0; i < 1442; i++)
            // {
            //     // Debug.Log(phis[i]);
            //     // phis[i] = (phis[i]+(2 * Math.PI)) % (2 * Math.PI);
            //     phis[i] = ((phis[i] % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
            //     // phis[i] = phis[i] % (2 * Math.PI);

            //     phis[i] = phis[i] - (((offsets[i] % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI));
            //     // phis[i] = phis[i] - ((offsets[i] + (2 * Math.PI)) % (2 * Math.PI));

            //     // phis[i] = phis[i] % (2 * Math.PI);

            //     phis[i] = phis[i] * phase_div / (2 * Math.PI);
            //     phis[i] = Math.Round(phis[i]);
            //     phis[i] = phis[i] >= 0 ? phis[i] : phase_div + phis[i];
            //     // Debug.Log(phis[i]);

            //     onOff[i] = 1;
            // }
        }
        else if (opt == 3)
        {
            List<Transducer> TArray = BottArray.ToList<Transducer>();
            TArray.AddRange(TopArray.ToList<Transducer>());

            // Vector3 P_control_loc = new Vector3(0f, 0f, 0.12f); // normal xyz

            // Vector3 P_control_loc = new Vector3(0f, -0.085f, 0.12f); // normal xyz FOR A
            // Vector3 P_control_loc = new Vector3(0f, 0.095f, 0.12f); // normal xyz FOR B

            // Vector3 P_control_loc = new Vector3(-0.075f, -0.065f, 0.12f); // normal xyz FOR Top C Bott A
            // Vector3 P_control_loc = new Vector3(-0.075f, 0.065f, 0.12f); // normal xyz FOR Top C Bott B

            // Vector3 P_control_loc = new Vector3(0.075f, -0.065f, 0.12f); // normal xyz FOR Bott C Top A

            // --------------- Changing solver size and center to match selected fpgas

            // Vector3 P_control_loc = new Vector3(0f, -0.071f, 0.122f); // normal xyz FOR A
            // Vector3 P_control_loc = new Vector3(0f, 0.0714f, 0.122f); // normal xyz FOR B

            // Vector3 P_control_loc = new Vector3(-0.062f,-0.036f, 0.122f); // normal xyz FOR Top C Bott A
            // Vector3 P_control_loc = new Vector3(-0.062f, 0.036f, 0.122f); // normal xyz FOR Top C Bott B

            // Vector3 P_control_loc = new Vector3(0.062f, -0.036f, 0.122f); // normal xyz FOR Bott C Top A
            // Vector3 P_control_loc = new Vector3(0.062f, 0.036f, 0.122f); // normal xyz FOR Bott C Top B

            // Vector3 P_control_loc = solvrCenterVector;
            Vector3 P_control_loc = new Vector3(0f, -0.071f, 0.122f);


            Focus focus = this.GetComponent<Focus>();
            double[] phases_f = focus.SimpleFocus(TArray, P_control_loc);


            bool fmode = true;
            if (fmode)
            {
                phases_f = focus.TwinTrap(TArray, phases_f, solvrSizeVector, solvrCenterVector, Math.PI / 2f);
            }
            else
            {
                phases_f = focus.VortexTrap(TArray, phases_f, solvrSizeVector, solvrCenterVector, 1);
            }

            phis = phases_f;
            onOff = new int[1442];

            for (int i = 0; i < 1442; i++)
            {
                // Debug.Log(phis[i]);
                // double d_offset = Math.Round((offsets[i] / Math.PI) * 16) % 32;
                // Debug.Log(d_offset);
                // phis[i] = Math.Round((phis[i] + ((-offsets[i]) / Math.PI)) * 16) % 32;
                phis[i] = Math.Round(((phis[i] + ((-offsets[i]) / Math.PI)) % 2) * 16) % 32;
                // phis[i] = Math.Round((phis[i]) * 16) % 32;

                // Debug.Log(phis[i]);
                while (phis[i] < 0)
                {
                    phis[i] += 32;
                }

                // System.Random random = new System.Random();
                // random.NextDouble();

                // Debug.Log(phis[i]);

                onOff[i] = 1;
                // if(i>721){
                //     onOff[i] = 0;
                // }
            }
        }
        else if (opt == 4)
        {
            List<Transducer> TArray = BottArray.ToList<Transducer>();
            TArray.AddRange(TopArray.ToList<Transducer>());

            Vector3 P_control_loc = new Vector3(0f, 0f, 0.12f); // normal xyz

            Focus focus = this.GetComponent<Focus>();
            double[] phases_f = focus.BerritTrap_2(TArray, P_control_loc, 1);


            phis = phases_f;
            onOff = new int[1442];

            for (int i = 0; i < 1442; i++)
            {
                // Debug.Log(phis[i]);
                // double d_offset = Math.Round((offsets[i] / Math.PI) * 16) % 32;
                double d_offset = offsets[i];
                // if(d_offset<0){
                //     d_offset = (2*Math.PI + d_offset);
                // }
                // Debug.Log(d_offset);
                // phis[i] = Math.Round((phis[i] + ((-offsets[i]) / Math.PI)) * 16) % 32;
                // phis[i] = Math.Round((phis[i]) * 16) % 32;
                // phis[i] = Math.Round((phis[i]+(2 * Math.PI)) % (2 * Math.PI));
                //
                phis[i] = (int)Math.Round(16 * ((phis[i] - d_offset) % (2 * Math.PI)) / Math.PI) % 32;

                Debug.Log(phis[i]);

                while (phis[i] < 0)
                {
                    phis[i] += 32;
                }

                // Debug.Log(phis[i]);

                onOff[i] = 1;
                // if(i>721){
                //     onOff[i] = 0;
                // }
            }

        }
        else
        {
            throw new Exception("Invalid Option");
            // phis = new double[]{};
            // onOff = new int[]{};
        }



        
        for (int i = 0; i < 721; i++)
        {
            int bott_ind = i;
            int top_ind = i + 721;

            // int bott_brd = 0;
            // int top_brd = 3;
            int bott_brd = BOTT_BRD;
            int top_brd = TOP_BRD;
            //no need now
            if (fpgaIndexArray[bott_ind] != bott_brd && BRD_DEBUG)
            {
                onOff[bott_ind] = 0;
            }

            if (fpgaIndexArray[top_ind] != top_brd && BRD_DEBUG)
            {
                onOff[top_ind] = 0;
            }

            // onOff[bott_ind] = 0;

            // Debug.Log(4*onOff[bott_ind]);

            if (SERIALON)
            {
                int amp = 4; // 4
                // if ((fpgaIndexArray[bott_ind] == bott_brd && BRD_DEBUG) || !BRD_DEBUG)
                {
                    var retrytime1 = Time.realtimeSinceStartup;
                    (int, int) resp1 = SetPixel(FPGA_Serial[fpgaIndexArray[bott_ind]], inBankIndexArray[bott_ind], amp * onOff[bott_ind], (int)phis[bott_ind]);

                    while (resp1 != (amp * onOff[bott_ind], (int)phis[bott_ind]))
                    {
                        while (Time.realtimeSinceStartup - retrytime1 < 1) { }
                        resp1 = SetPixel(FPGA_Serial[fpgaIndexArray[bott_ind]], inBankIndexArray[bott_ind], amp * onOff[bott_ind], (int)phis[bott_ind]);
                    }
                }

                // if ((fpgaIndexArray[top_ind] == top_brd && BRD_DEBUG) || !BRD_DEBUG)
                {
                    var retrytime2 = Time.realtimeSinceStartup;
                    (int, int) resp2 = SetPixel(FPGA_Serial[fpgaIndexArray[top_ind]], inBankIndexArray[top_ind], amp * onOff[top_ind], (int)phis[top_ind]);

                    while (resp2 != (amp * onOff[top_ind], (int)phis[top_ind]))
                    {
                        while (Time.realtimeSinceStartup - retrytime2 < 1) { }
                        resp2 = SetPixel(FPGA_Serial[fpgaIndexArray[top_ind]], inBankIndexArray[top_ind], amp * onOff[top_ind], (int)phis[top_ind]);
                    }
                }

            }
        }


        Debug.Log("Time taken for solver and serial output : " + (Time.realtimeSinceStartup - temptime));
    }

    private (int, int) SetPixel(SerialPort ser, int address, int duty, int phase)
    {
        string set_cmd = string.Format("CD{0:X4}{1:X2}{2:X2}", address, duty, phase);
        byte[] byteArray = StringToByteArray(set_cmd);
        ser.Write(byteArray, 0, 5);
        return GetPixel(ser, address);
    }

    private (int, int) GetPixel(SerialPort ser, int address)
    {
        // Debug.Log("Attempting to get Pixel");
        string get_cmd = string.Format("EF{0:X4}", address);
        byte[] byteArray = StringToByteArray(get_cmd);
        ser.Write(byteArray, 0, 3);
        int ack0 = ser.ReadByte();
        int ack1 = ser.ReadByte();
        return (ack0, ack1);


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
