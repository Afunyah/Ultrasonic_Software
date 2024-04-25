using UnityEngine;
using UnityEditor;
using System.Linq;
using System.Collections.Generic;

public static class DebugMenu
{
    [MenuItem("Debug/Print Global Position")]
    public static void PrintGlobalPosition()
    {
        if (Selection.activeGameObject != null)
        {
            Debug.Log(Selection.activeGameObject.name + " is at " + Selection.activeGameObject.transform.position);
        }
    }

    // [MenuItem("Debug/Print Transducer Distances")]
    // public static void CalcTransducers()
    // {
    //     GhostParticle ghost1 = GameObject.Find("Sphere_1").AddComponent<GhostParticle>();
    //     GhostParticle ghost2 = GameObject.Find("Sphere_2").AddComponent<GhostParticle>();
    //     float HexCntr_z = GameObject.Find("HexCenter").GetComponent<Transform>().transform.localPosition.y;
    //     List<Transform> selectedObjects = Selection.transforms.ToList();

    //     // 303, 213, 224
    //     // 304, 214, 225, 235
    //     // 291, 184, 290, 183, 289
    //     // 189, 101, 114, 125
    //     // 190, 102, 115
    //     List<int> trsnums = new List<int> { 303, 213, 224, 304, 214, 225, 235, 291, 184, 290, 183, 289, 189, 101, 114, 125, 190, 102, 115 };
    //     List<Transform> transducers = new List<Transform> { };
    //     foreach (int item in trsnums)
    //     {
    //         transducers.Add(GameObject.Find(string.Concat("HexLev/BottomPlate/BottomTransducers/Transducer.", item.ToString())).GetComponent<Transform>());
    //     }

    //     selectedObjects = transducers;

    //     List<GhostTransducerPositionData> gtpdatlist = new List<GhostTransducerPositionData> { };
    //     foreach (Transform trsf in selectedObjects)
    //     {
    //         Transducer tr = trsf.gameObject.AddComponent<Transducer>();
    //         tr.Init(0, 0, HexCntr_z);

    //         GhostTransducerPositionData gtpdat = new GhostTransducerPositionData(tr, ghost1.GetComponent<GhostParticle>(), ghost2.GetComponent<GhostParticle>());
    //         gtpdatlist.Add(gtpdat);
    //     }

    //     gtpdatlist = gtpdatlist.OrderBy(x => x.GetDist()).ToList();

    //     foreach (GhostTransducerPositionData gtpdat in gtpdatlist)
    //     {
    //         Debug.Log(gtpdat.trs.name + " { " + gtpdat.GetDist() + ", " + gtpdat.ang + " }");
    //     }

    //     foreach (Transform trsf in selectedObjects)
    //     {
    //         GameObject.DestroyImmediate(trsf.GetComponent<Transducer>());
    //         GameObject.DestroyImmediate(trsf.GetComponent<BoxCollider>());
    //         GameObject.DestroyImmediate(trsf.GetComponent<SphereCollider>());
    //         GameObject.DestroyImmediate(trsf.GetComponent<GhostParticle>());
    //     }

    //     Debug.DrawLine(ghost1.GetPostion(), ghost2.GetPostion(), Color.red, 10);
    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_1").GetComponent<GhostParticle>());
    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_2").GetComponent<GhostParticle>());

    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_1").GetComponent<BoxCollider>());
    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_2").GetComponent<SphereCollider>());

    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_1").GetComponent<BoxCollider>());
    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_2").GetComponent<SphereCollider>());

    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_1").GetComponent<Transducer>());
    //     GameObject.DestroyImmediate(GameObject.Find("Sphere_2").GetComponent<Transducer>());
    // }
}







// using UnityEngine;
// using UnityEditor;
// using System.Linq;

// public static class DebugMenu
// {
//     static GhostTransducerPositionData gtpdat;
//     static GhostParticle ghost1;
//     static GhostParticle ghost2;

//     [MenuItem("Debug/Print Global Position")]
//     public static void PrintGlobalPosition()
//     {
//         if (Selection.activeGameObject != null)
//         {
//             Debug.Log(Selection.activeGameObject.name + " is at " + Selection.activeGameObject.transform.position);
//         }
//     }

//     [MenuItem("Debug/Print Transducer Distances")]
//     public static void CalcTransducers()
//     {
//         ghost1 = GameObject.Find("Sphere_1").GetComponent<GhostParticle>();
//         ghost2 = GameObject.Find("Sphere_2").GetComponent<GhostParticle>();
//         float HexCntr_z = GameObject.Find("HexCenter").GetComponent<Transform>().transform.localPosition.y;
//         Transform[] selectedObjects = Selection.transforms;

//         foreach (Transform trsf in selectedObjects)
//         {
//             if(trsf.gameObject.name == "Sphere_1"){

//             }else if (trsf.gameObject.name == "Sphere_2"){

//             }
//             else if (trsf.gameObject.name.Contains("Transducer")){

//             }
//             Transducer tr = trsf.gameObject.AddComponent<Transducer>();
//             tr.Init(0, 0, HexCntr_z);

//             gtpdat = new GhostTransducerPositionData(tr, ghost1.GetComponent<GhostParticle>(), ghost2.GetComponent<GhostParticle>());
//             Debug.Log("here");
//             Debug.Log(gtpdat.trs.name + " { " + gtpdat.GetDist() + ", " + gtpdat.ang + " }");

//         }
//         Debug.DrawLine(ghost1.GetPostion(), ghost2.GetPostion());
//     }
// }