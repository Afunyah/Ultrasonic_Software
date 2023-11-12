using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using UnityEngine;


public class FlowHandler : MonoBehaviour
{
    public Material normalMaterial;
    public Material selectedMaterial;
    public new Camera camera;

    private GameObject selected;
    private bool isSelected;

    private readonly string selTag = "SELECTED";
    private readonly string unselTag = "Untagged";

    public GameObject uiCanvas;

    private GraphicRaycaster m_Raycaster;
    private PointerEventData m_PointerEventData;
    private EventSystem m_EventSystem;


    void Start()
    {
        isSelected = false;
    }

    void Update()
    {
        // go over isselected logic. Defined here but also assigned throughout below
        isSelected = (GameObject.FindWithTag("SELECTED")!=null);
        if (Input.GetMouseButton(0))
        {
            Ray ray = camera.ScreenPointToRay(Input.mousePosition);
            // Check for collider hit
            if (Physics.Raycast(ray, out RaycastHit hitInfo))
            {
                // Specifically check for levparticle hit
                if (hitInfo.collider.gameObject.GetComponent<SelectParticle>() != null)
                {
                    // Deselect old particle and select particle hit
                    if (isSelected && selected != hitInfo.collider.gameObject)
                    {
                        selected.GetComponent<Renderer>().material = normalMaterial;
                        selected.tag = unselTag;
                    }

                    selected = hitInfo.collider.gameObject;
                    selected.GetComponent<Renderer>().material = selectedMaterial;
                    selected.tag = selTag;
                    isSelected = true;
                }
                // Any other collider is hit
                else if (isSelected)
                {
                    isSelected = false;
                    selected.tag = unselTag;
                    selected.GetComponent<Renderer>().material = normalMaterial;
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
                        // Debug.Log("Hit " + uiComp.gameObject.name);
                        switch (uiCompName)
                        {
                            case "AddButton":
                                break;
                            case "RemoveButton":
                                if (isSelected)
                                {
                                    Debug.Log("Particle can be Removed!");
                                }
                                break;
                            case "MoveButton":
                                if (isSelected)
                                {
                                    Debug.Log("Particle can be Moved!");
                                }
                                break;
                            default:
                                Debug.Log("Null");
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
                        selected.tag = unselTag;
                        selected.GetComponent<Renderer>().material = normalMaterial;
                    }
                }

            }
        }

    }
}
