
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

    private TextAsset solver_fpga_csv;
    private TextAsset solver_unity_csv;

    private TextAsset solver_xytest_csv;

    private string[] solver_fpga_coords;
    private string[] solver_unity_coords;

    private string[] solver_xytest_coords;

    private TextAsset offsets_csv;

    private string[] offsets_text;

    static private double[] offsets;
    static private int[] xyTestIndexArray;

    static private int[] solverIndexArray;
    static private int[] fpgaIndexArray;
    static private int[] inBankIndexArray;
    static private int[] unityIndexArray;


    static private List<Transducer> TArray;


    void Awake()
    {
        int n = 1442;
        solverIndexArray = new int[n];
        fpgaIndexArray = new int[n];
        inBankIndexArray = new int[n];
        unityIndexArray = new int[n];
        xyTestIndexArray = new int[n];

        offsets = new double[n]; 
        

        TArray = new List<Transducer>();

        solver_fpga_csv = Resources.Load<TextAsset>("solver_fpga_coords");
        solver_unity_csv = Resources.Load<TextAsset>("solver_unity_coords");
        solver_xytest_csv = Resources.Load<TextAsset>("solver_xytest_index");

        offsets_csv = Resources.Load<TextAsset>("offsets_bott_top_rad");

        // Splitting the dataset in the end of line
        solver_fpga_coords = solver_fpga_csv.text.Split('\n');
        solver_unity_coords = solver_unity_csv.text.Split('\n');
        solver_xytest_coords = solver_xytest_csv.text.Split('\n');

        offsets_text = offsets_csv.text.Split('\n');

        // Iterating through the split dataset to split into rows
        for (int i = 0; i < n; i++)
        {
            string[] fpga_row = solver_fpga_coords[i].Split(',');
            solverIndexArray[i] = Int32.Parse(fpga_row[0]);
            fpgaIndexArray[i] = Int32.Parse(fpga_row[1]);
            inBankIndexArray[i] = Int32.Parse(fpga_row[2]);

            unityIndexArray[i] = Int32.Parse(solver_unity_coords[i]);

            xyTestIndexArray[i] = Int32.Parse(solver_xytest_coords[i]);

            offsets[i] = Double.Parse(offsets_text[i]);
        }


    }
    

    static public int[] GetSolverIndexArray(){
        return solverIndexArray;
    }

    static public int[] GetFpgaIndexArray(){
        return fpgaIndexArray;
    }

    static public int[] GetInBankIndexArray(){
        return inBankIndexArray;
    }

    static public int[] GetUnityIndexArray(){
        return unityIndexArray;
    }

    static public int[] GetXYTestIndexArray(){
        return xyTestIndexArray;
    }

    static public double[] GetOffsets(){
        return offsets;
    }



    void Update()
    {

    }


    static public List<Transducer> GetTArray()
    {
        return TArray;
    }
}

