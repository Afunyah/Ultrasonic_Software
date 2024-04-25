// using System.Collections;
// using System.Collections.Generic;
// using System.Numerics;
// using UnityEngine;

// using Vector3 = UnityEngine.Vector3;
// using Vector2 = UnityEngine.Vector2;
// using System;
// using Unity.VisualScripting;

// public class Labeller : MonoBehaviour
// {
//     // Start is called before the first frame update

//     private double[] hexCoords;
//     private List<Vector3> mapperMoves;
//     private Transducer curr_transducer;

//     private Vector3 toMove;
//     private int moveCount;

//     private bool firstCollision;

//     private float ColliderRadius;

//     private TextAsset csvdata;
//     private string[] splitData;

//     private List<int> solverIndexList;
//     private List<int> fpgaIndexList;
//     private List<int> inBankIndexList;

//     private float firstRealDistance;
//     private float firstSolverDistance;

//     private Vector3 TransducerPos_0;
//     private Vector3 TransducerPos_1;

//     private float tdist;


//     static private List<Transducer> TArray;

//     Transducer curr_tr;
//     Transducer old_tr;
//     bool isTransducer;
//     void Awake()
//     {
//         ColliderRadius = this.GetComponent<SphereCollider>().radius * this.transform.localScale.x;
//     }

//     private Rigidbody TreeRigidbody;
//     private BoxCollider TreeBoxCollider;

//     void Start()
//     {
//         isTransducer = false;
//         // TreeRigidbody = GetComponent<Rigidbody>();
//         // TreeBoxCollider = GetComponent<BoxCollider>();

//         old_tr = null;
//         tdist = 0.3f;

//         firstRealDistance = 0;
//         firstSolverDistance = 0;

//         TransducerPos_0 = Vector3.zero;
//         TransducerPos_1 = Vector3.zero;

//         solverIndexList = new List<int>();
//         fpgaIndexList = new List<int>();
//         inBankIndexList = new List<int>();

//         TArray = new List<Transducer>();

//         csvdata = Resources.Load<TextAsset>("output");

//         // Splitting the dataset in the end of line
//         splitData = csvdata.text.Split('\n');
//         Debug.Log(splitData.Length);
//         // Iterating through the split dataset in order to spli into rows
//         for (int i = 1; i < 722; i++)
//         {
//             // Debug.Log(splitData[i]);
//             string[] row = splitData[i].Split(',');
//             // solverIndexList.Add(int.Parse(row[0]));
//             // fpgaIndexList.Add(int.Parse(row[1]));
//             // inBankIndexList.Add(int.Parse(row[3]));
//             // Debug.Log(row[0].GetType());
//             // solverIndexList.Add(Convert.ToInt32(row[0]));
//             // fpgaIndexList.Add(Convert.ToInt32(row[1]));
//             // inBankIndexList.Add(Convert.ToInt32(row[3]));
//             // Debug.Log(row);
//             solverIndexList.Add(Int32.Parse(row[0]));
//             fpgaIndexList.Add(Int32.Parse(row[1]));
//             inBankIndexList.Add(Int32.Parse(row[3]));
//         }


//         firstCollision = true;
//         mapperMoves = new List<Vector3>();
//         moveCount = 0;
//         toMove = Vector3.zero;

//         hexCoords = Solver.GetHexCoords();


//         for (int i = 0; i <= 600; i++)
//         {
//             float x_coord_curr = (float)hexCoords[i];
//             float y_coord_curr = (float)hexCoords[i + 721];

//             float x_coord_next = (float)hexCoords[i + 1];
//             float y_coord_next = (float)hexCoords[i + 721 + 1];

//             Vector3 curr = new Vector3(x_coord_curr, 0, y_coord_curr);
//             Vector3 next = new Vector3(x_coord_next, 0, y_coord_next);

//             Vector3 traverse = next - curr;
//             Vector3 traverse_unit = Vector3.Normalize(traverse);

//             mapperMoves.Add(traverse_unit);

//             if (i == 0)
//             {
//                 float firstSolverDistance = Vector3.Magnitude(traverse);
//             }
//         }
//     }



//     void OnTriggerEnter(Collider objectName)
//     {
//         Debug.Log("Entered collision with " + objectName.gameObject.name);
//         // TreeRigidbody.isKinematic = false;
//         // TreeBoxCollider.isTrigger = false;
//         if(objectName.GetComponent<Transducer>() != null){
//             isTransducer = true;
//             curr_tr = objectName.GetComponent<Transducer>();
//         }
//         else{
//             isTransducer = false;
//         }
//     }

//     void OnTriggerExit(Collider objectName){
//         if(objectName.GetComponent<Transducer>() != null){
//             old_tr = objectName.GetComponent<Transducer>();
//         }
//     }


//     void Update()
//     {
//         // 90 out actually
//         if (moveCount <= 600)
//         {
//             if(isTransducer){

//                 // if(old_tr == null){
//                 //     normal
//                 // }

//                 if(old_tr != null && old_tr.name!=curr_tr.name){
//                     old_tr = curr_tr;
//                     moveCount++;
//                 }

//                 // if(old_tr != null && old_tr.name==curr_tr.name){

//                 // }else{
//                 //     Debug.Log("Hr");
//                 //     moveCount++;
//                 // }

//             }
//             // Collider[] forcedCollisions = Physics.OverlapSphere(this.transform.position, ColliderRadius);
//             // foreach (Collider hitColliders in forcedCollisions)
//             // {
//                 // this.gameObject.SendMessage("OnTriggerStay", hitColliders);
//                 // if (hitColliders.GetComponent<Transducer>() != null)
//                 // {

//                 //     Transducer trs = hitColliders.GetComponent<Transducer>();
//                 //     // Debug.Log(trs.name);

//                 //     if (firstCollision)
//                 //     {
//                 //         firstCollision = false;
//                 //         curr_transducer = trs;

//                 //         trs.SetSolverIndex(solverIndexList[moveCount]);
//                 //         trs.SetFpgaIndex(fpgaIndexList[moveCount]);
//                 //         trs.SetInBankIndex(inBankIndexList[moveCount]);

//                 //         float x_coord_curr = (float)hexCoords[moveCount];
//                 //         float y_coord_curr = (float)hexCoords[moveCount + 721];
//                 //         if (trs.GetPlate() == 0)
//                 //         {
//                 //             trs.SetSolverPostion(new Vector3(x_coord_curr, y_coord_curr, 0));
//                 //         }
//                 //         else
//                 //         {
//                 //             trs.SetSolverPostion(new Vector3(x_coord_curr, y_coord_curr, tdist));
//                 //         }

//                 //         TArray.Add(trs);
//                 //     }
//                 //     else
//                 //     {
//                 //         if (trs.name == curr_transducer.name)
//                 //         {
//                 //             // Debug.Log("same");
//                 //             if (moveCount == 0)
//                 //             {
//                 //                 TransducerPos_0 = curr_transducer.GetPosition();
//                 //             }
//                 //             if (moveCount == 1)
//                 //             {
//                 //                 TransducerPos_1 = curr_transducer.GetPosition();
//                 //             }
//                 //         }
//                 //         else
//                 //         {
//                 //             curr_transducer = trs;
//                 //             moveCount++;
//                 //             TArray.Add(trs);
//                 //             Debug.Log("HERE");
//                 //         }
//                 //     }


//                 // }
//             // }

//             this.transform.position += mapperMoves[moveCount] * 0.5f * Time.deltaTime;

//             // firstRealDistance = Vector3.Magnitude(TransducerPos_0 - TransducerPos_1);
//             // Solver.solvScale = (firstRealDistance / firstSolverDistance);
//         }
//         else
//         {

//         }
//     }


//     // Update is called once per frame
//     // void Update()
//     // {
//     //     // 90 out actually
//     //     if (moveCount <= 600)
//     //     {
//     //         Collider[] forcedCollisions = Physics.OverlapSphere(this.transform.position, ColliderRadius);
//     //         foreach (Collider hitColliders in forcedCollisions)
//     //         {
//     //             // this.gameObject.SendMessage("OnTriggerStay", hitColliders);
//     //             if (hitColliders.GetComponent<Transducer>() != null)
//     //             {

//     //                 Transducer trs = hitColliders.GetComponent<Transducer>();
//     //                 // Debug.Log(trs.name);

//     //                 if (firstCollision)
//     //                 {
//     //                     firstCollision = false;
//     //                     curr_transducer = trs;

//     //                     trs.SetSolverIndex(solverIndexList[moveCount]);
//     //                     trs.SetFpgaIndex(fpgaIndexList[moveCount]);
//     //                     trs.SetInBankIndex(inBankIndexList[moveCount]);

//     //                     float x_coord_curr = (float)hexCoords[moveCount];
//     //                     float y_coord_curr = (float)hexCoords[moveCount + 721];
//     //                     if (trs.GetPlate() == 0)
//     //                     {
//     //                         trs.SetSolverPostion(new Vector3(x_coord_curr, y_coord_curr, 0));
//     //                     }
//     //                     else
//     //                     {
//     //                         trs.SetSolverPostion(new Vector3(x_coord_curr, y_coord_curr, tdist));
//     //                     }

//     //                     TArray.Add(trs);
//     //                 }
//     //                 else
//     //                 {
//     //                     if (trs.name == curr_transducer.name)
//     //                     {
//     //                         // Debug.Log("same");
//     //                         if (moveCount == 0)
//     //                         {
//     //                             TransducerPos_0 = curr_transducer.GetPosition();
//     //                         }
//     //                         if (moveCount == 1)
//     //                         {
//     //                             TransducerPos_1 = curr_transducer.GetPosition();
//     //                         }
//     //                     }
//     //                     else
//     //                     {
//     //                         curr_transducer = trs;
//     //                         moveCount++;
//     //                         TArray.Add(trs);
//     //                         Debug.Log("HERE");
//     //                     }
//     //                 }


//     //             }
//     //         }

//     //         this.transform.position += mapperMoves[moveCount] * 0.01f;

//     //         firstRealDistance = Vector3.Magnitude(TransducerPos_0 - TransducerPos_1);
//     //         Solver.solvScale = (firstRealDistance/firstSolverDistance);
//     //     }else{

//     //     }
//     // }


//     static public List<Transducer> GetTArray()
//     {
//         // List<Transducer> arr = new List<Transducer>();
//         // arr = TArray;
//         return TArray;
//     }
// }






using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;

using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;
using System;
using Unity.VisualScripting;

public class Labeller : MonoBehaviour
{
    // Start is called before the first frame update

private double[] hexCoords;

    private TextAsset csvdata;
    private TextAsset ucoords;
    private string[] splitData;
    private string[] splitData_2;

    public List<int> solverIndexList;
    public List<int> fpgaIndexList;
    public List<int> inBankIndexList;
    public List<int> uCoordsList;


    static private List<Transducer> TArray;

    // void Awake()
    // {
    // }


    void Awake()
    {
        solverIndexList = new List<int>();
        fpgaIndexList = new List<int>();
        inBankIndexList = new List<int>();

        TArray = new List<Transducer>();

        csvdata = Resources.Load<TextAsset>("output");
        ucoords = Resources.Load<TextAsset>("somecoords_2");

        // Splitting the dataset in the end of line
        splitData = csvdata.text.Split('\n');
        splitData_2 = ucoords.text.Split('\n');

        // Debug.Log(splitData.Length);
        // Iterating through the split dataset in order to spli into rows
        for (int i = 1; i < 722; i++)
        {
            // Debug.Log(splitData[i]);
            string[] row = splitData[i].Split(',');
            // solverIndexList.Add(int.Parse(row[0]));
            // fpgaIndexList.Add(int.Parse(row[1]));
            // inBankIndexList.Add(int.Parse(row[3]));
            // Debug.Log(row[0].GetType());
            // solverIndexList.Add(Convert.ToInt32(row[0]));
            // fpgaIndexList.Add(Convert.ToInt32(row[1]));
            // inBankIndexList.Add(Convert.ToInt32(row[3]));
            // Debug.Log(row);
            solverIndexList.Add(Int32.Parse(row[0]));
            fpgaIndexList.Add(Int32.Parse(row[1]));
            inBankIndexList.Add(Int32.Parse(row[3]));
            uCoordsList.Add(Int32.Parse(splitData_2[i-1]));

        }


    }
    

    public List<int> GetSolverIndexList(){
        return solverIndexList;
    }

    public List<int> GetFpgaIndexList(){
        return fpgaIndexList;
    }

    public List<int> GetInBankIndexList(){
        return inBankIndexList;
    }

    public List<int> GetUCoordsIndexList(){
        return uCoordsList;
    }


    void Update()
    {

    }






    static public List<Transducer> GetTArray()
    {
        // List<Transducer> arr = new List<Transducer>();
        // arr = TArray;
        return TArray;
    }
}

