using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine;
using System.Xml.Schema;
using System.Numerics;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;
using System.Runtime.InteropServices;
using TMPro;

/// <summary>
/// Manages the workflow of the program.
/// Graphic Raycasters are used to determine interaction with UI elements such as menus and buttons, as well selecting and moving particles.
/// </summary>
public class FlowHandler : MonoBehaviour
{


    double[] phses = new double[1442];

    /// <summary>
    /// Camera reference to main HexCam. Provided within the Unity editor.
    /// </summary>
    public new Camera camera;

    /// <summary>
    /// UI Canvas element for buttons and text. Provided within the Unity editor. 
    /// </summary>
    public GameObject uiCanvas;

    /// <summary>
    /// GraphicRaycaster element of the uiCanvas. Used to detect user hits on graphic elements.
    /// </summary>
    private GraphicRaycaster m_Raycaster;

    /// <summary>
    /// Stores interaction elements as events. Provided within the Unity Editor.
    /// </summary>
    private EventSystem m_EventSystem;

    /// <summary>
    /// Stores mouse pointer information.
    /// </summary>
    private PointerEventData m_PointerEventData;

    /// <summary>
    /// References the currently selected particle as a Gameobject.
    /// </summary>
    private GameObject selected;

    /// <summary>
    /// References the currently selected particle as a LevParticle.
    /// </summary>
    private LevParticle SelectedLevParticle;

    /// <summary>
    /// Holds the Renderer properties of the selected LevParticle.
    /// </summary>
    private Renderer SelectedRenderer;

    /// <summary>
    /// Material asset for specifying the visual properties of a normal unselected LevParticle. Provided within the Unity editor.
    /// </summary>
    public Material normalMaterial;

    /// <summary>
    /// Material asset for specifying the visual properties of the selected LevParticle. Provided within the Unity editor.
    /// </summary>
    public Material selectedMaterial;

    public Material TransducerMaterial;

    /// <summary>
    /// Keeps track of whether a LevParticle is selected or not.
    /// </summary>
    private bool isSelected;

    /// <summary>
    /// Specifies the transducer layer as set in Unity. This layer is excluded from the raycaster. 
    /// </summary>
    private int trLayer;

    /// <summary>
    /// Coordinates of the start point for the selected LevParticle's trajectory.
    /// </summary>
    private Vector3 spoint;

    /// <summary>
    /// Coordinates of the end point for the selected LevParticle's trajectory.
    /// </summary>
    private Vector3 epoint;

    /// <summary>
    /// Keeps track of whether a trajectory is being created or not.
    /// </summary>
    private bool isCreatingTraj;

    public TMP_InputField DebugInputField1;
    public TMP_InputField DebugInputField2;
    public TMP_InputField DebugInputField3_1;
    public TMP_InputField DebugInputField3_2;
    public TMP_InputField DebugInputField4;

    public TMP_InputField DebugInputField5_1;
    public TMP_InputField DebugInputField5_2;

    private int d1_value;
    private int d2_value;

    private int d3_1_value;
    private int d3_2_value;

    private int d4_value;

    // public int[] counterArray;
    private bool uia_prog_flag;
    private int uia_counter;
    private float settime;
    private bool uia_pause;



    void Awake()
    {
        d1_value = 0;
        d2_value = 0;

        d3_1_value = 0;
        d3_2_value = 0;

        d4_value = 497;

    }
    void Start()
    {
        DebugInputField1.onEndEdit.AddListener(DebugInputField1_func);
        DebugInputField2.onEndEdit.AddListener(DebugInputField2_func);
        DebugInputField3_1.onEndEdit.AddListener(DebugInputField3_1_func);
        DebugInputField3_2.onEndEdit.AddListener(DebugInputField3_2_func);

        DebugInputField4.onEndEdit.AddListener(DebugInputField4_func);

        isSelected = false;
        isCreatingTraj = false;
        trLayer = 1 << 6; // Set Transducer Layer

        // counterArray = this.GetComponent<StateInit>().unityIndexArray;
        uia_prog_flag = true;
        settime = 0;
        uia_counter = 0;
        uia_pause = true;

    }


    void Update()
    {
        int[] counterArray = this.GetComponent<StateInit>().unityIndexArray;
        int[] testArray = this.GetComponent<StateInit>().xyTestIndexArray;
        Transducer[] bottarr = this.GetComponent<StateInit>().BottArray;
        Transducer[] toparr = this.GetComponent<StateInit>().BottArray;
        int pt = 0;
        int sm = 0;

        if (uia_pause || uia_counter >= 1442)
        {

        }
        else if (uia_prog_flag)
        {
            uia_prog_flag = false;

            sm = 0;
            
            if (uia_counter >= 721)
            {
                pt = 1;
                UIA_setter(counterArray[uia_counter], sm, pt);
                // UIA_setter(toparr[testArray[uia_counter - 721] - 1].GetUnityIndex(), sm, pt);
            }
            else
            {
                pt = 0;
                UIA_setter(counterArray[uia_counter], sm, pt);
                // UIA_setter(bottarr[testArray[uia_counter] - 1].GetUnityIndex(), sm, pt);
            }

            uia_counter++;
            sm = 1;

            
            if(uia_counter >= 1442){

            }
            else if (uia_counter >= 721)
            {
                pt = 1;
                UIA_setter(counterArray[uia_counter], sm, pt);
                // UIA_setter(toparr[testArray[uia_counter - 721] - 1].GetUnityIndex(), sm, pt);
            }
            else
            {
                pt = 0;
                UIA_setter(counterArray[uia_counter], sm, pt);
                // UIA_setter(bottarr[testArray[uia_counter] - 1].GetUnityIndex(), sm, pt);
            }

            settime = Time.realtimeSinceStartup;
        }
        else if (Time.realtimeSinceStartup - settime > 0.005)
        {
            uia_prog_flag = true;
        }

        DebugInputField5_1.text = (uia_counter + 1).ToString();

        // DebugInputField5_2.text = counterArray[uia_counter].ToString();
        if (uia_counter >= 1442)
        {

        }
        else if (uia_counter >= 721)
        {
            DebugInputField5_2.text = toparr[uia_counter - 721].GetUnityIndex().ToString();
        }
        else
        {
            DebugInputField5_2.text = bottarr[uia_counter].GetUnityIndex().ToString();
        }


        // Hit Handler
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = camera.ScreenPointToRay(Input.mousePosition);
            // Check for collider hit
            if (Physics.Raycast(ray, out RaycastHit hitInfo, 100f, ~trLayer))
            {
                // Specifically check for levparticle hit
                if (hitInfo.collider.gameObject.GetComponent<LevParticle>() != null)
                {
                    // Deselect old particle and select particle hit
                    if (isSelected && selected != hitInfo.collider.gameObject)
                    {
                        SelectedLevParticle.SetSelect(false);
                        SelectedRenderer.material = normalMaterial;
                        isCreatingTraj = false;
                    }

                    // Get properties of selected particle
                    isSelected = true;
                    selected = hitInfo.collider.gameObject;
                    SelectedLevParticle = selected.GetComponent<LevParticle>();
                    SelectedRenderer = selected.GetComponent<Renderer>();
                    SelectedLevParticle.SetSelect(isSelected);
                    SelectedRenderer.material = selectedMaterial;
                }
                // Any other collider is hit
                else if (isSelected)
                {
                    isSelected = false;
                    SelectedLevParticle.SetSelect(isSelected);
                    SelectedRenderer.material = normalMaterial;
                }
            }
            else
            {
                // No collider is hit, check for UI hit
                m_Raycaster = uiCanvas.GetComponent<GraphicRaycaster>();
                m_EventSystem = GetComponent<EventSystem>();

                m_PointerEventData = new PointerEventData(m_EventSystem);
                m_PointerEventData.position = Input.mousePosition;

                List<RaycastResult> uiHitList = new List<RaycastResult>();
                m_Raycaster.Raycast(m_PointerEventData, uiHitList);

                // Filter hit on UI components
                if (uiHitList.Count != 0)
                {
                    string uiCompName;
                    foreach (RaycastResult uiComp in uiHitList)
                    {
                        uiCompName = uiComp.gameObject.name;
                        switch (uiCompName)
                        {
                            case "AddButton":
                                break;
                            case "RemoveButton":
                                if (isSelected) { SelectedLevParticle.DeleteParticle(); }
                                isSelected = false;
                                break;
                            case "MoveButton":
                                if (isSelected)
                                {
                                }
                                // this.GetComponent<StateInit>().TestFunc(tval);
                                this.GetComponent<StateInit>().UpdateLevState();
                                // this.GetComponent<Solver>().Solve();
                                break;
                            case "D1on":
                                this.GetComponent<StateInit>().D1TestFunc(d1_value, 1);
                                break;
                            case "D1off":
                                this.GetComponent<StateInit>().D1TestFunc(d1_value, 0);
                                break;
                            case "D2on":
                                this.GetComponent<StateInit>().D2TestFunc(d2_value, 1);
                                break;
                            case "D2off":
                                this.GetComponent<StateInit>().D2TestFunc(d2_value, 0);
                                break;
                            case "D3on":
                                this.GetComponent<StateInit>().D3TestFunc(d3_1_value, d3_2_value, 1);
                                break;
                            case "D3off":
                                this.GetComponent<StateInit>().D3TestFunc(d3_1_value, d3_2_value, 0);
                                break;
                            case "D4on":
                                D4_update(1);
                                break;
                            case "D4off":
                                D4_update(0);
                                break;
                            case "D4prev":
                                D4_update(2);
                                break;
                            case "D4next":
                                D4_update(3);
                                break;
                            case "D5pause":
                                uia_pause = !uia_pause;
                                break;
                            case "StartSerial":
                                if (!this.GetComponent<StateInit>().SERIALON)
                                {
                                    this.GetComponent<StateInit>().StartSerial();
                                }
                                break;
                            case "xDown":
                                if (isSelected) { SelectedLevParticle.MoveX(-1); }
                                break;
                            case "xUp":
                                if (isSelected) { SelectedLevParticle.MoveX(1); }
                                break;
                            case "yDown":
                                if (isSelected) { SelectedLevParticle.MoveY(-1); }
                                break;
                            case "yUp":
                                if (isSelected) { SelectedLevParticle.MoveY(1); }
                                break;
                            case "zDown":
                                if (isSelected) { SelectedLevParticle.MoveZ(-1); }
                                break;
                            case "zUp":
                                if (isSelected) { SelectedLevParticle.MoveZ(1); }
                                break;
                            case "StartButton":
                                if (isSelected)
                                {
                                    // Read startpoint and enter trajectory mode
                                    spoint = SelectedLevParticle.GetPosition();
                                    isCreatingTraj = true;
                                }
                                break;
                            case "EndButton":
                                if (isSelected && isCreatingTraj)
                                {
                                    // Read endpoint and create trajectory
                                    epoint = SelectedLevParticle.GetPosition();
                                    SelectedLevParticle.AddTrajectory(spoint, epoint);
                                    isCreatingTraj = false;
                                }
                                break;
                            default:
                                break;
                        }
                    }

                }
                else
                {
                    // No Collider or UI is hit
                    if (isSelected)
                    {
                        isSelected = false;
                        SelectedLevParticle.SetSelect(isSelected);
                        SelectedRenderer.material = normalMaterial;
                        isCreatingTraj = false;
                    }
                }

            }
        }

    }

    public void DebugInputField1_func(string text)
    {
        try
        {
            int val;
            val = Int32.Parse(text);
            // if((val < 0) || (val > 32)){
            //     throw new Exception("Out of range");
            // }

            d1_value = val;
        }
        catch (Exception ex) { Debug.Log(ex); }
    }

    public void DebugInputField2_func(string text)
    {
        try
        {
            int val;
            val = Int32.Parse(text);
            // if((val < 0) || (val > 32)){
            //     throw new Exception("Out of range");
            // }

            d2_value = val;
        }
        catch (Exception ex) { Debug.Log(ex); }
    }

    public void DebugInputField3_1_func(string text)
    {
        try
        {
            int val;
            val = Int32.Parse(text);
            // if((val < 0) || (val > 32)){
            //     throw new Exception("Out of range");
            // }

            d3_1_value = val;
        }
        catch (Exception ex) { Debug.Log(ex); }
    }

    public void DebugInputField3_2_func(string text)
    {
        try
        {
            int val;
            val = Int32.Parse(text);
            // if((val < 0) || (val > 32)){
            //     throw new Exception("Out of range");
            // }

            d3_2_value = val;
        }
        catch (Exception ex) { Debug.Log(ex); }
    }


    public void DebugInputField4_func(string text)
    {
        try
        {
            int val;
            val = Int32.Parse(text);
            // if((val < 0) || (val > 32)){
            //     throw new Exception("Out of range");
            // }

            d4_value = val;
        }
        catch (Exception ex) { Debug.Log(ex); }
    }

    public void D4_update(int tk)
    {
        try
        {
            String d4str = "";

            if (d4_value < 10)
            {
                d4str = "00" + d4_value.ToString();
            }
            else if (d4_value < 100)
            {
                d4str = "0" + d4_value.ToString();
            }
            else
            {
                d4str = d4_value.ToString();
            }

            String tr = "HexLev/BottomPlate/BottomTransducers/Transducer." + d4str;
            if (tk == 0)
            {
                GameObject.Find(tr).GetComponent<Renderer>().material = TransducerMaterial;
            }
            else if (tk == 1)
            {
                GameObject.Find(tr).GetComponent<Renderer>().material = selectedMaterial;
            }
            else if (tk == 2)
            {
                d4_value--;
                DebugInputField4.text = d4_value.ToString();
            }
            else if (tk == 3)
            {
                d4_value++;
                DebugInputField4.text = d4_value.ToString();
            }

        }

        catch (Exception ex) { Debug.Log(ex); }
    }

    public void UIA_setter(int x, int tk, int pt)
    {
        try
        {
            String tr;
            int xval = x;


            if (pt == 1)
            {
                tr = "HexLev/TopPlate/TopTransducers/Transducer.";
                // x -= 721;
            }
            else
            {
                tr = "HexLev/BottomPlate/BottomTransducers/Transducer.";
            }

            String xstr = "";

            if (x < 10)
            {
                xstr = "00" + x.ToString();
            }
            else if (x < 100)
            {
                xstr = "0" + x.ToString();
            }
            else
            {
                xstr = x.ToString();
            }

            tr += xstr;


            if (tk == 0)
            {
                GameObject.Find(tr).GetComponent<Renderer>().material = normalMaterial;
            }
            else if (tk == 1)
            {
                GameObject.Find(tr).GetComponent<Renderer>().material = selectedMaterial;
            }

        }

        catch (Exception ex) { Debug.Log(ex); }
    }



}
