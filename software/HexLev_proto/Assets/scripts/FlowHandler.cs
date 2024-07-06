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
    private int d1_value;
    private int d2_value;

    private int d3_1_value;
    private int d3_2_value;


    void Awake()
    {
        d1_value = 0;
        d2_value = 0;

        d3_1_value = 0;
        d3_2_value = 0;

    }
    void Start()
    {
        DebugInputField1.onEndEdit.AddListener(DebugInputField1_func);
        DebugInputField2.onEndEdit.AddListener(DebugInputField2_func);
        DebugInputField3_1.onEndEdit.AddListener(DebugInputField3_1_func);
        DebugInputField3_2.onEndEdit.AddListener(DebugInputField3_2_func);

        isSelected = false;
        isCreatingTraj = false;
        trLayer = 1 << 6; // Set Transducer Layer
    }

    void Update()
    {
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
                            case "StartSerial":
                                if(!this.GetComponent<StateInit>().SERIALON){
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

}
