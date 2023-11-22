using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class DebugWindow : EditorWindow
{
    string myString = "Hello World";
    bool groupEnabled;
    bool myBool = true;
    float myFloat = 1.23f;

    private bool trSelected;
    private bool prtSelected;
    public GameObject ghost1;
    public GameObject ghost2;

    private Transform[] selectedObjects;

    [MenuItem("Window/Debug Window")]
    public static void ShowWindow()
    {
        EditorWindow.GetWindow(typeof(DebugWindow));
    }
    void Awake()
    {
        trSelected = false;
        prtSelected = false;
    }

    void OnGUI()
    {
        GUILayout.Label("Base Settings", EditorStyles.boldLabel);
        myString = EditorGUILayout.TextField("Text Field", myString);

        groupEnabled = EditorGUILayout.BeginToggleGroup("Optional Settings", groupEnabled);
        myBool = EditorGUILayout.Toggle("Toggle", myBool);
        myFloat = EditorGUILayout.Slider("Slider", myFloat, -3, 3);
        EditorGUILayout.EndToggleGroup();

        trSelected = EditorGUILayout.LinkButton("Select Transducers");
        prtSelected = EditorGUILayout.LinkButton("Select Particles");
    }

    void Update()
    {
        if (trSelected)
        {
            selectedObjects = Selection.transforms;
            this.CalcTransducers();
            trSelected = false;
        }
        if (prtSelected)
        {
            prtSelected = false;
        }
    }

    private void CalcTransducers()
    {
        foreach (Transform trsf in selectedObjects)
        {
            Transducer tr = trsf.GetComponent<Transducer>();
            if (tr != null)
            {
                GhostTransducerPositionData gtpdat = new GhostTransducerPositionData(tr, ghost1.GetComponent<GhostParticle>(), ghost2.GetComponent<GhostParticle>());
                Debug.Log(gtpdat.trs.name + " { " + gtpdat.GetDist() + ", " + gtpdat.ang + " }");
            }
        }
    }

}