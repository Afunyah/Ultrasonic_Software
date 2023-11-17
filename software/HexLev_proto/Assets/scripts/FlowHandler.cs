using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine;
using System.Xml.Schema;


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

    void Start()
    {
        isSelected = false;
        trLayer = 1<<6;
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
                                if (isSelected) Debug.Log("Particle can be Moved!");
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
                    }
                }

            }
        }

    }
}
