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

public class FlowHandler : MonoBehaviour
{
    public new Camera camera;
    public GameObject uiCanvas;

    private GraphicRaycaster m_Raycaster;
    private PointerEventData m_PointerEventData;
    private EventSystem m_EventSystem;

    private GameObject selected;
    private LevParticle SelectedLevParticle;
    private Renderer SelectedRenderer;

    public Material normalMaterial;
    public Material selectedMaterial;
    private bool isSelected;

    private int trLayer;

    private Vector3 spoint;
    private Vector3 epoint;
    private bool isCreatingTraj;

    private Vector2 tr;
    private Vector2 g1;
    private Vector2 g2;

    void Start()
    {
        isSelected = false;
        isCreatingTraj = false;
        trLayer = 1 << 6;
        // tr = new Vector2(new Vector3(72.49f, 4.07f, 33.25f).x,new Vector3(72.49f, 4.07f, 33.25f).z);
        // g1 = new Vector2(new Vector3(72.43f, 6.01f, 33.25f).x,new Vector3(72.43f, 6.01f, 33.25f).z);
        // g2 = new Vector2(new Vector3(72.49f, 6.01f, 33.25f).x,new Vector3(72.49f, 6.01f, 33.25f).z);

        // Debug.Log(g1-g2);
        // Debug.Log(tr-g2);
        // Debug.Log(Vector2.SignedAngle(g1-g2,tr-g2));
    }

    void Update()
    {
        // go over isselected logic. Defined here but also assigned throughout below
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
                                    Debug.Log("Particle can be Moved!");
                                    List<List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)>> FTTDList = SelectedLevParticle.GetFullTrajectoryTransducerDataList();

                                    foreach (List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> item in FTTDList)
                                    {
                                        foreach ((List<GhostTransducerPositionData>, List<GhostTransducerPositionData>) gtp in item)
                                        {

                                            Debug.Log("P2-T1 Data:");
                                            foreach (GhostTransducerPositionData gtpdat in gtp.Item1)
                                            {
                                                Debug.Log(gtpdat.trs.name + " { " + gtpdat.dist + ", " + gtpdat.ang + " }");
                                            }
                                            Debug.Log("P2-T2 Data:");
                                            foreach (GhostTransducerPositionData gtpdat in gtp.Item2)
                                            {
                                                Debug.Log(gtpdat.trs.name + " { " + gtpdat.dist + ", " + gtpdat.ang + " }");
                                            }


                                        }


                                    }
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
                                    spoint = SelectedLevParticle.GetPosition();
                                    isCreatingTraj = true;
                                }
                                break;
                            case "EndButton":
                                if (isSelected && isCreatingTraj)
                                {
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
}
