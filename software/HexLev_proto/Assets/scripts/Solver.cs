using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;
using Random = UnityEngine.Random;

using MathNet.Numerics;
using NLoptNet;

using Accord.Math;
using Accord;
using Accord.Math.Optimization;
using UnityEngine.AI;


public class Solver : MonoBehaviour
{
    // static public float solvScale = 0;

    private double f;
    private double c;

    private double lmbda;
    static public double tdist;

    private double[] coords;
    void Awake()
    {
        f = 40000;
        c = 343;

        lmbda = c / f;
        // tdist = 4 * lmbda;
        // tdist = 4 * lmbda;
        // tdist = (lmbda / 2) * 56;
        // tdist = 0.163f;
        tdist = 0.244f;
        // tdist = 0.324f;
    }

    void Start()
    {

    }

    void Update()
    {
    }

    public (double[], List<bool>) Solve()
    {
        List<Transducer> TArray = this.gameObject.GetComponent<StateInit>().BottArray.ToList<Transducer>();
        TArray.AddRange(this.gameObject.GetComponent<StateInit>().TopArray.ToList<Transducer>());

        // List<Transducer> TArray = this.gameObject.GetComponent<StateInit>().BottArray_tmp;
        // TArray.AddRange(this.gameObject.GetComponent<StateInit>().TopArray_tmp);

        List<float> dists_from_point;
        dists_from_point = new List<float>();

        Vector3 P_control_loc = new Vector3(0, 0, 0.12f);
        // Vector3 P_control_loc = new Vector3(0.1f, 0, (float)tdist / 2f);
        // Debug.Log((float)tdist/2);

        for (int i = 0; i < TArray.Count; i++)
        {
            Vector3 trSolvPos = TArray[i].GetSolverPostion();
            // float d = Vector2.Distance(new Vector2(trSolvPos.x, trSolvPos.y), new Vector2(P_control_loc.x, P_control_loc.y)); // y z swapped
            float d = Vector2.Distance(new Vector2(trSolvPos.x, trSolvPos.y), new Vector2(P_control_loc.x, P_control_loc.y));
            dists_from_point.Add(d);
            // Debug.Log(trSolvPos.ToString("F4"));
        }

        // float active_radius = 0.055f;
        float active_radius = 0.25f;

        List<bool> active_mask;
        active_mask = new List<bool>();

        foreach (float dist in dists_from_point)
        {
            active_mask.Add(dist <= active_radius);
        }

        // active_mask.AddRange(active_mask);

        List<Transducer> Tarray_active;
        Tarray_active = new List<Transducer>();

        for (int i = 0; i < active_mask.Count; i++)
        {

            if (active_mask[i])
            {
                Tarray_active.Add(TArray[i]);
                // Debug.Log(TArray[i].GetSolverPostion().z);
            }
            // if (active_mask[i])
            // {
            //     Debug.Log(1);
            // }else{
            //     Debug.Log(0);
            // }

        }

        int N_active = Tarray_active.Count;

        double V = 4 / 3 * Math.PI * 0.001f;
        double C0 = 343;
        double Cp = 2350;
        double r0 = 1.293;
        double rp = 20;


        double wfreq = f * 2 * Math.PI;
        double K1 = 0.25 * V * ((1 / (Math.Pow(C0, 2) * r0)) - (1 / (Math.Pow(Cp, 2) * rp)));
        double K2 = 0.75 * V * (r0 - rp) / (Math.Pow(wfreq, 2) * r0 * (r0 + 2 * rp));

        double wp = 1;
        double wx = 10;
        double wy = 10;
        double wz = 600;
        // double wp = 0.1;
        // double wx = 0.1;
        // double wy = 0.1;
        // double wz = 0.1;
        // double wp = 1f;
        // double wx = 1f;
        // double wy = 1f;
        // double wz = 0.1f;

        double[] K;
        K = new double[2];

        K[0] = K1;
        K[1] = K2;



        List<double> phi;
        phi = new List<double>();

        for (int i = 0; i < N_active; i++)
        {
            float randnum = Random.Range(0.0f, 1.0f);
            phi.Add(randnum * 2f * Math.PI);

            // phi.Add(2 * Math.PI);

            // double cb0 = Tarray_active[i].GetPhaseOffset();
            // cb0 = cb0 < 0 ? 0 : cb0;
            // phi.Add(cb0);

            // Debug.Log(phi[i]);
        }


        List<double> x = new List<double>();
        List<double> y = new List<double>();
        List<double> z = new List<double>();

        double dx = 0.01;
        double dy = 0.01;
        double dz = 0.01;

        List<double> delta;
        delta = new List<double> { dx, dy, dz };

        for (int i = -1; i < 3; i++)
        {
            x.Add(i * dx + P_control_loc[0]);
            y.Add(i * dy + P_control_loc[1]);
            z.Add(i * dz + P_control_loc[2]);
        }

        List<List<double>> calculation_points = new List<List<double>>();
        calculation_points = CombVec3(new List<List<double>>() { x, y, z });

        // foreach (var i in calculation_points)
        // {
        //     Debug.Log(i[0].ToString("F4") + " " + i[1].ToString("F4") + " " + i[2].ToString("F4"));
        // }

        int L = calculation_points.Count();


        List<DummyParticle> calculation_P;
        calculation_P = new List<DummyParticle>();

        for (int i = 0; i < L; i++)
        {
            DummyParticle lp = new DummyParticle(new Vector3((float)calculation_points[i][0], (float)calculation_points[i][1], (float)calculation_points[i][2]));
            calculation_P.Add(lp);
        }

        int[] bitmapArray = { 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        Complex[,] calcF_NOT = TransmissionMatrix(calculation_P, Tarray_active, GetSolverScale(TArray[0], TArray[1]));

        for (int i = 0; i < calcF_NOT.GetLength(0); i++)
        {
            for (int j = 0; j < calcF_NOT.GetLength(1); j++)
            {
                // Debug.Log(calcF_NOT[i, j]);
            }
        }

        // foreach (var i in calcF_NOT)
        // {
        //     String srng = "";

        //     srng += (i.ToString());
        //     srng += " ";

        //     Debug.Log(srng);
        // }

        List<List<Complex>> calcF;
        calcF = new List<List<Complex>>();
        for (int i = 0; i < calcF_NOT.GetLength(0); i++)
        {
            List<Complex> tmp = new List<Complex>();
            for (int j = 0; j < calcF_NOT.GetLength(1); j++)
            {
                tmp.Add(calcF_NOT[i, j] * bitmapArray[i]);
            }
            calcF.Add(tmp);
        }

        // foreach (var i in calcF)
        // {
        //     String srng = "";
        //     foreach (var item in i)
        //     {
        //         // srng += (item.Imaginary.ToString("F4"));
        //         Debug.Log(item.Real.ToString("F4"));
        //         // srng += " ";
        //     }
        //     // Debug.Log(srng);
        // }


        // SpatialDerivatives spatialDerivatives_debug = new SpatialDerivatives(phi.ToList<double>(), calcF);
        // Gorkov gorkov_debug = new Gorkov(spatialDerivatives_debug, K);
        // gorkov_debug.ComputeLaplacian();
        // double O_debug = wp * gorkov_debug.pAbs - wx * gorkov_debug.Uxx - wy * gorkov_debug.Uyy - wz * gorkov_debug.Uzz;
        // // Debug.Log(O_debug);

        // int N_debug = phi.Count;
        // double[] g_debug = new double[N_debug];

        // for (int i = 0; i < N_debug; i++)
        // {
        //     gorkov_debug.ComputeGradientLaplacian(i);
        //     g_debug[i] = wp * gorkov_debug.gpAbs - wx * gorkov_debug.gUxx - wy * gorkov_debug.gUyy - wz * gorkov_debug.gUzz;
        //     Debug.Log(g_debug[i].ToString("F4"));
        // }

        Func<double[], double> f1 = (x) =>
        {
            SpatialDerivatives spatialDerivatives = new SpatialDerivatives(x.ToList<double>(), calcF);
            Gorkov gorkov = new Gorkov(spatialDerivatives, K);

            gorkov.ComputeLaplacian();

            double O = wp * gorkov.pAbs - wx * gorkov.Uxx - wy * gorkov.Uyy - wz * gorkov.Uzz;
            return O;
        };



        double[] g1(double[] x)
        {

            SpatialDerivatives spatialDerivatives = new SpatialDerivatives(x.ToList<double>(), calcF);
            Gorkov gorkov = new Gorkov(spatialDerivatives, K);

            gorkov.ComputeLaplacian();

            int N = x.Length;
            double[] g = new double[N];


            for (int i = 0; i < N; i++)
            {
                gorkov.ComputeGradientLaplacian(i);
                g[i] = wp * gorkov.gpAbs - wx * gorkov.gUxx - wy * gorkov.gUyy - wz * gorkov.gUzz;
            }

            return g;
        }

        // BroydenFletcherGoldfarbShanno lbfgs = new BroydenFletcherGoldfarbShanno(numberOfVariables: phi.Count, function: f1, gradient: g1);
        ConjugateGradient lbfgs = new ConjugateGradient(numberOfVariables: phi.Count, function: f1, gradient: g1);

        // lbfgs.FunctionTolerance = 1e1;
        // lbfgs.MaxLineSearch = 1000;
        // BoundedBroydenFletcherGoldfarbShanno lbfgs = new BoundedBroydenFletcherGoldfarbShanno(numberOfVariables: phi.Count, function: f1, gradient: g1);
        // lbfgs.FunctionTolerance = 1e-10;
        // lbfgs.GradientTolerance = 0.1;
        // lbfgs.MaxIterations = 1000;
        // lbfgs.Progress += (s, e) =>
        // {
        //     Debug.Log(lbfgs.Value.ToString("F10"));
        // };

        bool success = false;
        success = lbfgs.Minimize(phi.ToArray<double>());
        double[] solution = lbfgs.Solution;
        if (success)
        {
            Debug.Log("Solver solution: <color=green>" + success + "</color>\tValue: " + lbfgs.Value);
            Debug.Log("Status: <color=green>" + lbfgs.Status + "</color>\tNsols: " + solution.Length);
        }
        else
        {
            Debug.Log("Solver solution: <color=red>" + success + "</color>\tValue: " + lbfgs.Value);
            Debug.Log("Status: <color=red>" + lbfgs.Status + "</color>\tNsols: " + solution.Length);
        }
        // Debug.Log(lbfgs.Iterations);
        // Debug.Log(lbfgs.Evaluations);
        // Debug.Log(lbfgs.MaxIterations);
        // Debug.Log(lbfgs.Epsilon);
        foreach (var value in solution)
        {
            // if (value > 2 * Math.PI)
            // {
            // Debug.Log(value);
            // }
            // Debug.Log(value);
        }

        double[] phi_total = new double[1442];
        int active_ind = 0;
        for (int i = 0; i < 1442; i++)
        {
            if (active_mask[i])
            {
                phi_total[i] = solution[active_ind];
                active_ind++;
            }
        }
        return (phi_total, active_mask);
    }

    static public float GetSolverScale(Transducer one, Transducer two)
    {
        // Debug.Log("(" + one.GetSolverPostion().x + ", " + one.GetSolverPostion().y + ", " + one.GetSolverPostion().z + ")" + " :: "+ "(" + two.GetSolverPostion().x + ", " + two.GetSolverPostion().y + ", " + two.GetSolverPostion().z + ")");
        // Debug.Log("(" + one.GetPosition().x + ", " + one.GetPosition().y + ", " + one.GetPosition().z + ")" + " :: "+ "(" + two.GetPosition().x + ", " + two.GetPosition().y + ", " + two.GetPosition().z + ")");
        float sp = Vector3.Distance(one.GetSolverPostion(), two.GetSolverPostion());
        float tp = Vector3.Distance(one.GetPosition(), two.GetPosition());
        // Debug.Log("(" + sp + ", " + tp + ")");
        return sp / tp;

    }

    static public double GetTdist()
    {
        return tdist;
    }

    static public double[] GetHexCoords()
    {
        // double T_diam = 9.8e-3;
        double T_diam = 10e-3;

        double spacing = 1e-3;
        double u = (T_diam + spacing) / 2.0;

        //     double[] x = { -0.0054f, -0.0108f, -0.0054f, 0.0054f, 0.0108f,
        // 0.0054, 0.0093530743608719377f, 0.0f, -0.0093530743608719377f,
        // -0.0093530743608719377f, 0.0f, 0.0093530743608719377f };

        double[] x = { -1, -2, -1, 1, 2, 1, Math.Sqrt(3), 0, -Math.Sqrt(3), -Math.Sqrt(3), 0, Math.Sqrt(3) };

        double[] i_data = new double[2];
        double[] b_ring_cords_data = new double[180];
        double[] ring_cords_data = new double[180];
        double[] coords = new double[1442];

        double s_ind = 2.0f;
        double e_ind = 7.0f;

        int i; int j; int b_i; int k; int rowIdx;

        double buffer_data_idx_0;

        coords[0] = 0;
        coords[721] = 0.0;

        i_data[1] = 0.0;
        for (i = 0; i < 15; i++)
        {
            int ring_cords_size_idx_0 = 6 * (i + 1);
            int colIdx = -1;
            for (j = 0; j < 2; j++)
            {
                rowIdx = -1;
                colIdx++;
                for (b_i = 0; b_i < 6; b_i++)
                {
                    for (k = 0; k <= i; k++)
                    {
                        ring_cords_data[((rowIdx + k) + ring_cords_size_idx_0 * colIdx) + 1] = u * x[b_i + 6 * j];
                    }

                    rowIdx = (rowIdx + i) + 1;
                }
            }

            for (k = 0; k < 2; k++)
            {
                for (rowIdx = 0; rowIdx <= ring_cords_size_idx_0 - 2; rowIdx++)
                {
                    colIdx = rowIdx + ring_cords_size_idx_0 * k;
                    ring_cords_data[colIdx + 1] += ring_cords_data[colIdx];
                }
            }

            i_data[0] = ((double)i + 1.0f) * 2.0f * u;
            for (j = 0; j < 2; j++)
            {
                for (rowIdx = 0; rowIdx < ring_cords_size_idx_0; rowIdx++)
                {
                    b_ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] =
                      ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] + i_data[j];
                }
            }

            for (j = 0; j < 2; j++)
            {
                for (rowIdx = 0; rowIdx < ring_cords_size_idx_0; rowIdx++)
                {
                    ring_cords_data[rowIdx + ring_cords_size_idx_0 * j] =
                      b_ring_cords_data[rowIdx + ring_cords_size_idx_0 * j];
                }
            }


            if (s_ind > e_ind)
            {
                j = 1;
            }
            else
            {
                j = (int)s_ind;
            }

            for (b_i = 0; b_i < 2; b_i++)
            {
                colIdx = b_i * ring_cords_size_idx_0;
                buffer_data_idx_0 = ring_cords_data[(colIdx + ring_cords_size_idx_0) - 1];
                for (k = ring_cords_size_idx_0; k >= 2; k--)
                {
                    rowIdx = colIdx + k;
                    ring_cords_data[rowIdx - 1] = ring_cords_data[rowIdx - 2];
                }

                ring_cords_data[colIdx] = buffer_data_idx_0;
            }

            colIdx = ring_cords_size_idx_0 << 1;
            for (k = 0; k < colIdx; k++)
            {
                // ring_cords_data[k] = rt_roundd_snf(ring_cords_data[k]);
                ring_cords_data[k] = Math.Round(ring_cords_data[k], 5);
            }

            for (rowIdx = 0; rowIdx < 2; rowIdx++)
            {
                for (colIdx = 0; colIdx < ring_cords_size_idx_0; colIdx++)
                {
                    coords[((j + colIdx) + 721 * rowIdx) - 1] = ring_cords_data[colIdx +
                      ring_cords_size_idx_0 * rowIdx];
                }
            }

            s_ind += ((double)i + 1.0f) * 6.0f;
            e_ind += (((double)i + 1.0f) + 1.0f) * 6.0f;
        }

        // for (i = 0; i < 20; i++)
        // {
        //     Debug.Log(coords[i]);
        // }

        return coords;
    }


    private Complex[,] TransmissionMatrix(List<DummyParticle> Parray, List<Transducer> Tarray, float ss)
    {
        double f = 40000;
        double c = 343;
        double lambda = c / f;
        double k = 2 * Math.PI / lambda;

        int L = Parray.Count;
        int N = Tarray.Count;

        // List<List<Complex>> F = new List<List<Complex>>();

        Complex[,] F = new Complex[L, N];

        for (int l = 0; l < L; l++)
        {
            DummyParticle particle = Parray[l];

            for (int n = 0; n < N; n++)
            {
                Transducer transducer = Tarray[n];
                Vector3 tCoord_real = transducer.GetPosition();
                Vector3 tCoord = transducer.GetSolverPostion();
                int tz = transducer.GetPlate() == 0 ? 1 : -1;
                Vector3 tNorm = new Vector3(0, 0, tz);


                // float d = particle.GetDistanceFromX(tCoord_real);
                // d /= ss;
                // float theta = particle.GetZAngleFromX(tCoord_real);

                // Debug.Log(tCoord.z);
                float d = particle.GetDistanceFromX(tCoord);
                // Debug.Log(d.ToString("F4"));

                double theta = particle.GetZAngleFromX(tCoord, tNorm);
                // Debug.Log(theta.ToString("F4"));

                double r = transducer.GetRadius();
                double Pref = transducer.GetRefPressure();

                // Debug.Log(Pref.ToString("F4"));


                double P = MathNet.Numerics.SpecialFunctions.BesselJ(0, k * r * Math.Sin(theta)) * Pref / d;
                if (double.IsNaN(P))
                {
                    P = Pref / d;
                }

                Complex Phi = new Complex(Math.Cos(k * d), Math.Sin(k * d));
                // Debug.Log(Phi.Imaginary.ToString("F4"));

                F[l, n] = P * Phi;
                // Debug.Log(P.ToString("F4"));
            }
        }

        return F;
    }


    static List<List<double>> CombVec(List<List<double>> arrays)
    {
        List<List<double>> result = new List<List<double>>();
        List<double> currentCombination = new List<double>();

        GenerateCombinations(arrays, 0, currentCombination, result); // counts up
        // GenerateCombinations2(arrays, 3, currentCombination, result); // counts down
        return result;
    }

    static void GenerateCombinations(List<List<double>> arrays, int currentIndex, List<double> currentCombination, List<List<double>> result)
    {
        if (currentIndex == arrays.Count)
        {
            result.Add(new List<double>(currentCombination));
            return;
        }

        foreach (double num in arrays[currentIndex])
        {
            currentCombination.Add(num);
            GenerateCombinations(arrays, currentIndex + 1, currentCombination, result);
            currentCombination.RemoveAt(currentCombination.Count - 1);
        }
    }


    static void GenerateCombinations2(List<List<double>> arrays, int currentIndex, List<double> currentCombination, List<List<double>> result)
    {
        if (currentIndex == 0)
        {
            result.Add(new List<double>(currentCombination));
            return;
        }

        try
        {
            foreach (double num in arrays[currentIndex - 1])
            {
                // Debug.Log(currentCombination.Count);
                currentCombination.Add(num);
                GenerateCombinations2(arrays, currentIndex - 1, currentCombination, result);
                currentCombination.RemoveAt(currentCombination.Count - 1);


            }
        }
        catch (Exception ex)
        {
            Debug.Log(ex);

            Debug.Log(currentCombination.Count);
            Debug.Log(currentIndex);
        }
    }

    static List<List<double>> CombVec3(List<List<double>> arrays)
    {
        List<List<double>> result = new List<List<double>>(64);
        List<double> currentCombination = new List<double>(3);

        GenerateCombinations3(arrays, result); // counts up
        // GenerateCombinations2(arrays, 3, currentCombination, result); // counts down
        return result;
    }

    static void GenerateCombinations3(List<List<double>> arrays, List<List<double>> result)
    {
        for (int i = 0; i < 64; i++)
        {
            result.Add(new List<double>(3));
            result[i].Add(arrays[0][i % 4]);
        }

        for (float i = 0; i < 16; i++)
        {
            float j = 0;
            while (j < 4)
            {
                try { result[(int)((4 * i) + j)].Add(arrays[1][(int)Mathf.Floor(i % 4)]); }
                catch (DivideByZeroException) { result[(int)((4 * i) + j)].Add(arrays[1][0]); }
                j++;
            }
        }

        for (float i = 0; i < 4; i++)
        {
            float j = 0;
            while (j < 16)
            {
                try { result[(int)((16 * i) + j)].Add(arrays[2][(int)Mathf.Floor(i % 4)]); }
                catch (DivideByZeroException) { result[(int)((16 * i) + j)].Add(arrays[2][0]); }
                j++;
            }
        }

    }

}

public class SpatialDerivatives
{
    private readonly int N;

    public Complex[] p;
    public Complex[] px;
    public Complex[] py;
    public Complex[] pz;
    public Complex[] pxx;
    public Complex[] pxy;
    public Complex[] pxz;
    public Complex[] pyy;
    public Complex[] pyz;
    public Complex[] pzz;
    public Complex[] pxxx;
    public Complex[] pxyy;
    public Complex[] pxzz;
    public Complex[] pyxx;
    public Complex[] pyyy;
    public Complex[] pyzz;
    public Complex[] pzxx;
    public Complex[] pzyy;
    public Complex[] pzzz;


    public Complex P;
    public Complex Px;
    public Complex Py;
    public Complex Pz;
    public Complex Pxx;
    public Complex Pxy;
    public Complex Pxz;
    public Complex Pyy;
    public Complex Pyz;
    public Complex Pzz;
    public Complex Pxxx;
    public Complex Pxyy;
    public Complex Pxzz;
    public Complex Pyxx;
    public Complex Pyyy;
    public Complex Pyzz;
    public Complex Pzxx;
    public Complex Pzyy;
    public Complex Pzzz;

    public SpatialDerivatives(List<double> phi, List<List<Complex>> F)
    {
        this.N = phi.Count;

        this.p = new Complex[N];
        this.px = new Complex[N];
        this.py = new Complex[N];
        this.pz = new Complex[N];
        this.pxx = new Complex[N];
        this.pxy = new Complex[N];
        this.pxz = new Complex[N];
        this.pyy = new Complex[N];
        this.pyz = new Complex[N];
        this.pzz = new Complex[N];
        this.pxxx = new Complex[N];
        this.pxyy = new Complex[N];
        this.pxzz = new Complex[N];
        this.pyxx = new Complex[N];
        this.pyyy = new Complex[N];
        this.pyzz = new Complex[N];
        this.pzxx = new Complex[N];
        this.pzyy = new Complex[N];
        this.pzzz = new Complex[N];

        this.P = new Complex(0, 0);
        this.Px = new Complex(0, 0);
        this.Py = new Complex(0, 0);
        this.Pz = new Complex(0, 0);
        this.Pxx = new Complex(0, 0);
        this.Pxy = new Complex(0, 0);
        this.Pxz = new Complex(0, 0);
        this.Pyy = new Complex(0, 0);
        this.Pyz = new Complex(0, 0);
        this.Pzz = new Complex(0, 0);
        this.Pxxx = new Complex(0, 0);
        this.Pxyy = new Complex(0, 0);
        this.Pxzz = new Complex(0, 0);
        this.Pyxx = new Complex(0, 0);
        this.Pyyy = new Complex(0, 0);
        this.Pyzz = new Complex(0, 0);
        this.Pzxx = new Complex(0, 0);
        this.Pzyy = new Complex(0, 0);
        this.Pzzz = new Complex(0, 0);

        ComputeSpatialDerivatives(phi, F);
    }


    private void ComputeSpatialDerivatives(List<double> phi, List<List<Complex>> F)
    {
        double dx = 0.01;
        double dy = 0.01;
        double dz = 0.01;

        List<double> delta;
        delta = new List<double> { dx, dy, dz };
        List<Complex> tau = new List<Complex>();
        foreach (double elem in phi)
        {
            tau.Add(new Complex(Math.Cos(elem), Math.Sin(elem)));
            // Debug.Log(new Complex(Math.Cos(elem), Math.Sin(elem)).ToString("F4"));
        }

        // List<List<List<List<Complex>>>> PMatrix = new List<List<List<List<Complex>>>>(N); // Nx4x4x4
        Complex[,,,] PMatrix = new Complex[N, 4, 4, 4];

        List<Complex> Fcol = new List<Complex>(); // 64

        int debug_count = 1;

        // 64 or N here?
        for (int i = 0; i < N; i++)
        {
            Fcol = new List<Complex>(); // 64*
            // Fcol.AddRange(F[i]);

            for (int c = 0; c < 64; c++)
            {
                Fcol.Add(F[c][i]);
                // Debug.Log(debug_count + " : " + F[c][i].ToString("F4"));
                // debug_count++;
            }



            for (int ii = 0; ii < Fcol.Count; ii++)
            {
                Fcol[ii] = Complex.Multiply(Fcol[ii], tau[i]);
                // Debug.Log(debug_count + " : " + Fcol[ii].ToString("F4"));
                // debug_count++;
            }

            for (int j = 0; j < 4; j++)
            {
                for (int k = 0; k < 4; k++)
                {
                    for (int l = 0; l < 4; l++)
                    {
                        // PMatrix[i, j, k, l] = Fcol[4 * l + k];
                        PMatrix[i, j, k, l] = Fcol[(16 * j) + (4 * k) + l];
                        // Debug.Log(debug_count + " : " + PMatrix[i, j, k, l].ToString("F4"));
                        // debug_count++;
                        // Debug.Log(PMatrix[i, j, k, l].Imaginary.ToString("F4"));
                    }
                }
            }
        }

        // int x0 = 2;
        // int y0 = 2;
        // int z0 = 2;
        int x0 = 1;
        int y0 = 1;
        int z0 = 1;

        for (int i = 0; i < N; i++)
        {
            p[i] = PMatrix[i, z0, y0, x0];

            px[i] = PMatrix[i, z0, y0, x0 + 1] - PMatrix[i, z0, y0, x0];
            py[i] = PMatrix[i, z0, y0 + 1, x0] - PMatrix[i, z0, y0, x0];
            pz[i] = PMatrix[i, z0 + 1, y0, x0] - PMatrix[i, z0, y0, x0];
            pxx[i] = PMatrix[i, z0, y0, x0 + 1] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0, y0, x0 - 1];
            pyy[i] = PMatrix[i, z0, y0 + 1, x0] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0, y0 - 1, x0];
            pzz[i] = PMatrix[i, z0 + 1, y0, x0] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0 - 1, y0, x0];
            pxy[i] = PMatrix[i, z0, y0 + 1, x0 + 1] - PMatrix[i, z0, y0 - 1, x0 + 1] - PMatrix[i, z0, y0 + 1, x0 - 1] + PMatrix[i, z0, y0 - 1, x0 - 1];
            pxz[i] = PMatrix[i, z0 + 1, y0, x0 + 1] - PMatrix[i, z0 - 1, y0, x0 + 1] - PMatrix[i, z0 + 1, y0, x0 - 1] + PMatrix[i, z0 - 1, y0, x0 - 1];
            pyz[i] = PMatrix[i, z0 + 1, y0 + 1, x0] - PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + PMatrix[i, z0 - 1, y0 - 1, x0];
            pxxx[i] = PMatrix[i, z0, y0, x0 + 2] - 3 * PMatrix[i, z0, y0, x0 + 1] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0, x0 - 1];
            pyyy[i] = PMatrix[i, z0, y0 + 2, x0] - 3 * PMatrix[i, z0, y0 + 1, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0 - 1, x0];
            pzzz[i] = PMatrix[i, z0 + 2, y0, x0] - 3 * PMatrix[i, z0 + 1, y0, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0 - 1, y0, x0];
            // Complex dbg = new Complex();
            // dbg = (PMatrix[i, z0 + 2, y0, x0] - 3 * PMatrix[i, z0 + 1, y0, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0 - 1, y0, x0]);
            // Debug.Log(dbg.Real.ToString("F8"));
            pxyy[i] = PMatrix[i, z0, y0 + 1, x0 + 1] - 2 * PMatrix[i, z0, y0, x0 + 1] + PMatrix[i, z0, y0 - 1, x0 + 1] - PMatrix[i, z0, y0 + 1, x0 - 1] + 2 * PMatrix[i, z0, y0, x0 - 1] - PMatrix[i, z0, y0 - 1, x0 - 1];
            pxzz[i] = PMatrix[i, z0, y0, x0 + 1] - 2 * PMatrix[i, z0, y0, x0] + PMatrix[i, z0, y0, x0 - 1] - PMatrix[i, z0, y0, x0 + 1] + 2 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0, x0 - 1];
            pyxx[i] = PMatrix[i, z0, y0 + 1, x0 + 1] - 2 * PMatrix[i, z0, y0 + 1, x0] + PMatrix[i, z0, y0 + 1, x0 - 1] - PMatrix[i, z0, y0 - 1, x0 + 1] + 2 * PMatrix[i, z0, y0 - 1, x0] - PMatrix[i, z0, y0 - 1, x0 - 1];
            pyzz[i] = PMatrix[i, z0, y0 + 1, x0] - 2 * PMatrix[i, z0, y0, x0] + PMatrix[i, z0, y0 - 1, x0] - PMatrix[i, z0, y0 + 1, x0] + 2 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0 - 1, x0];
            pzxx[i] = PMatrix[i, z0 + 1, y0, x0 + 1] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0, x0 - 1] - PMatrix[i, z0 - 1, y0, x0 + 1] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0, x0 - 1];
            pzyy[i] = PMatrix[i, z0 + 1, y0 + 1, x0] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0 - 1, x0];
            // Complex dbg = new Complex();
            // dbg = (PMatrix[i, z0 + 1, y0 + 1, x0] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0 - 1, x0]);
            // Debug.Log(dbg.Real.ToString("F8"));
        }

        // for (int i = 0; i < N; i++)
        // {
        //     p[i] = PMatrix[i, z0, y0, x0];

        //     px[i] = (PMatrix[i, z0, y0, x0 + 1] - PMatrix[i, z0, y0, x0]) / delta[0];
        //     py[i] = (PMatrix[i, z0, y0 + 1, x0] - PMatrix[i, z0, y0, x0]) / delta[1];
        //     pz[i] = (PMatrix[i, z0 + 1, y0, x0] - PMatrix[i, z0, y0, x0]) / delta[2];
        //     pxx[i] = (PMatrix[i, z0, y0, x0 + 1] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0, y0, x0 - 1]) / Math.Pow(delta[0], 2);
        //     pyy[i] = (PMatrix[i, z0, y0 + 1, x0] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0, y0 - 1, x0]) / Math.Pow(delta[1], 2);
        //     pzz[i] = (PMatrix[i, z0 + 1, y0, x0] - 2 * PMatrix[i, z0, y0, x0] + 2 * PMatrix[i, z0 - 1, y0, x0]) / Math.Pow(delta[2], 2);
        //     pxy[i] = (PMatrix[i, z0, y0 + 1, x0 + 1] - PMatrix[i, z0, y0 - 1, x0 + 1] - PMatrix[i, z0, y0 + 1, x0 - 1] + PMatrix[i, z0, y0 - 1, x0 - 1]) / (4 * delta[0] * delta[1]);
        //     pxz[i] = (PMatrix[i, z0 + 1, y0, x0 + 1] - PMatrix[i, z0 - 1, y0, x0 + 1] - PMatrix[i, z0 + 1, y0, x0 - 1] + PMatrix[i, z0 - 1, y0, x0 - 1]) / (4 * delta[0] * delta[2]);
        //     pyz[i] = (PMatrix[i, z0 + 1, y0 + 1, x0] - PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + PMatrix[i, z0 - 1, y0 - 1, x0]) / (4 * delta[1] * delta[2]);
        //     pxxx[i] = (PMatrix[i, z0, y0, x0 + 2] - 3 * PMatrix[i, z0, y0, x0 + 1] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0, x0 - 1]) / Math.Pow(delta[0], 3);
        //     pyyy[i] = (PMatrix[i, z0, y0 + 2, x0] - 3 * PMatrix[i, z0, y0 + 1, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0 - 1, x0]) / Math.Pow(delta[1], 3);
        //     pzzz[i] = (PMatrix[i, z0 + 2, y0, x0] - 3 * PMatrix[i, z0 + 1, y0, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0 - 1, y0, x0]) / Math.Pow(delta[2], 3);
        //     // Complex dbg = new Complex();
        //     // dbg = (PMatrix[i, z0 + 2, y0, x0] - 3 * PMatrix[i, z0 + 1, y0, x0] + 3 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0 - 1, y0, x0]);
        //     // Debug.Log(dbg.Real.ToString("F8"));
        //     pxyy[i] = (PMatrix[i, z0, y0 + 1, x0 + 1] - 2 * PMatrix[i, z0, y0, x0 + 1] + PMatrix[i, z0, y0 - 1, x0 + 1] - PMatrix[i, z0, y0 + 1, x0 - 1] + 2 * PMatrix[i, z0, y0, x0 - 1] - PMatrix[i, z0, y0 - 1, x0 - 1]) / (2 * delta[0] * Math.Pow(delta[1], 2));
        //     pxzz[i] = (PMatrix[i, z0, y0, x0 + 1] - 2 * PMatrix[i, z0, y0, x0] + PMatrix[i, z0, y0, x0 - 1] - PMatrix[i, z0, y0, x0 + 1] + 2 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0, x0 - 1]) / (2 * delta[0] * Math.Pow(delta[2], 2));
        //     pyxx[i] = (PMatrix[i, z0, y0 + 1, x0 + 1] - 2 * PMatrix[i, z0, y0 + 1, x0] + PMatrix[i, z0, y0 + 1, x0 - 1] - PMatrix[i, z0, y0 - 1, x0 + 1] + 2 * PMatrix[i, z0, y0 - 1, x0] - PMatrix[i, z0, y0 - 1, x0 - 1]) / (2 * delta[1] * Math.Pow(delta[0], 2));
        //     pyzz[i] = (PMatrix[i, z0, y0 + 1, x0] - 2 * PMatrix[i, z0, y0, x0] + PMatrix[i, z0, y0 - 1, x0] - PMatrix[i, z0, y0 + 1, x0] + 2 * PMatrix[i, z0, y0, x0] - PMatrix[i, z0, y0 - 1, x0]) / (2 * delta[1] * Math.Pow(delta[2], 2));
        //     pzxx[i] = (PMatrix[i, z0 + 1, y0, x0 + 1] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0, x0 - 1] - PMatrix[i, z0 - 1, y0, x0 + 1] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0, x0 - 1]) / (2 * delta[2] * Math.Pow(delta[0], 2));
        //     pzyy[i] = (PMatrix[i, z0 + 1, y0 + 1, x0] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0 - 1, x0]) / (2 * delta[2] * Math.Pow(delta[1], 2));
        //     // Complex dbg = new Complex();
        //     // dbg = (PMatrix[i, z0 + 1, y0 + 1, x0] - 2 * PMatrix[i, z0 + 1, y0, x0] + PMatrix[i, z0 + 1, y0 - 1, x0] - PMatrix[i, z0 - 1, y0 + 1, x0] + 2 * PMatrix[i, z0 - 1, y0, x0] - PMatrix[i, z0 - 1, y0 - 1, x0]);
        //     // Debug.Log(dbg.Real.ToString("F8"));
        // }


        P = ComplexSum(p);
        Px = ComplexSum(px);
        Py = ComplexSum(py);
        Pz = ComplexSum(pz);
        Pxx = ComplexSum(pxx);
        Pxy = ComplexSum(pxy);
        Pxz = ComplexSum(pxz);
        Pyy = ComplexSum(pyy);
        Pyz = ComplexSum(pyz);
        Pzz = ComplexSum(pzz);
        Pxxx = ComplexSum(pxxx);
        Pxyy = ComplexSum(pxyy);
        Pxzz = ComplexSum(pxzz);
        Pyxx = ComplexSum(pyxx);
        Pyyy = ComplexSum(pyyy);
        Pyzz = ComplexSum(pyzz);
        Pzxx = ComplexSum(pzxx);
        Pzyy = ComplexSum(pzyy);
        Pzzz = ComplexSum(pzzz);

        double c1 = 1e-2;
        double c2 = 1e-4;
        // double c3 = 4e-4;
        double c3 = 1e-6;
        // double c5 = 2e-6;

        Px = Px / delta[0];
        Py = Py / delta[1];
        Pz = Pz / delta[2];

        Pxx = Pxx / Math.Pow(delta[0], 2);
        Pyy = Pyy / Math.Pow(delta[1], 2);
        Pzz = Pzz / Math.Pow(delta[2], 2);

        Pxy = Pxy / (4 * delta[0] * delta[1]);
        Pxz = Pxz / (4 * delta[0] * delta[2]);
        Pyz = Pyz / (4 * delta[1] * delta[2]);

        Pxxx = Pxxx / Math.Pow(delta[0], 3);
        Pyyy = Pyyy / Math.Pow(delta[1], 3);
        Pzzz = Pzzz / Math.Pow(delta[2], 3);

        Pxyy = Pxyy / (2 * delta[0] * Math.Pow(delta[1], 2));
        Pxzz = Pxzz / (2 * delta[0] * Math.Pow(delta[2], 2));
        Pyxx = Pyxx / (2 * delta[1] * Math.Pow(delta[0], 2));
        Pyzz = Pyzz / (2 * delta[1] * Math.Pow(delta[2], 2));
        Pzxx = Pzxx / (2 * delta[2] * Math.Pow(delta[0], 2));
        Pzyy = Pzyy / (2 * delta[2] * Math.Pow(delta[1], 2));

        //

        // Px = Px * c1 / delta[0];
        // Py = Py * c1 / delta[1];
        // Pz = Pz * c1 / delta[2];

        // Pxx = Pxx * c2 / Math.Pow(delta[0], 2);
        // Pyy = Pyy * c2 / Math.Pow(delta[1], 2);
        // Pzz = Pzz * c2 / Math.Pow(delta[2], 2);

        // Pxy = Pxy * c2 / (4 * delta[0] * delta[1]);
        // Pxz = Pxz * c2 / (4 * delta[0] * delta[2]);
        // Pyz = Pyz * c2 / (4 * delta[1] * delta[2]);

        // Pxxx = Pxxx * c3 / Math.Pow(delta[0], 3);
        // Pyyy = Pyyy * c3 / Math.Pow(delta[1], 3);
        // Pzzz = Pzzz * c3 / Math.Pow(delta[2], 3);

        // Pxyy = Pxyy * c3 / (2 * delta[0] * Math.Pow(delta[1], 2));
        // Pxzz = Pxzz * c3 / (2 * delta[0] * Math.Pow(delta[2], 2));
        // Pyxx = Pyxx * c3 / (2 * delta[1] * Math.Pow(delta[0], 2)); 
        // Pyzz = Pyzz * c3 / (2 * delta[1] * Math.Pow(delta[2], 2));
        // Pzxx = Pzxx * c3 / (2 * delta[2] * Math.Pow(delta[0], 2));
        // Pzyy = Pzyy * c3 / (2 * delta[2] * Math.Pow(delta[1], 2));

        // Debug.Log(P.Real.ToString("F8"));
        // Debug.Log(Px.Real.ToString("F8"));
        // Debug.Log(Py.Real.ToString("F8"));
        // Debug.Log(Pz.Real.ToString("F8"));
        // Debug.Log(Pxx.Real.ToString("F8"));
        // Debug.Log(Pxy.Real.ToString("F8"));
        // Debug.Log(Pxz.Real.ToString("F8"));
        // Debug.Log(Pyy.Real.ToString("F8"));
        // Debug.Log(Pyz.Real.ToString("F8"));
        // Debug.Log(Pzz.Real.ToString("F8"));
        // Debug.Log(Pxxx.Real.ToString("F8"));
        // Debug.Log(Pxyy.Real.ToString("F8"));
        // Debug.Log(Pxzz.Real.ToString("F8"));
        // Debug.Log(Pyxx.Real.ToString("F8"));
        // Debug.Log(Pyyy.Real.ToString("F8"));
        // Debug.Log(Pyzz.Real.ToString("F8"));
        // Debug.Log(Pzxx.Real.ToString("F8"));
        // Debug.Log(Pzyy.Real.ToString("F8"));
        // Debug.Log(Pzzz.Real.ToString("F8"));
    }



    private Complex ComplexSum(Complex[] complex_list)
    {
        Complex result = new Complex(0, 0);
        foreach (Complex comp in complex_list)
        {
            result += comp;
        }
        return result;
    }


}


public class Gorkov
{
    public double Uxx;
    public double Uyy;
    public double Uzz;
    public double gUxx;
    public double gUyy;
    public double gUzz;

    public double gpAbs;
    public double pAbs;

    private readonly SpatialDerivatives sd;
    private readonly double[] K;


    private readonly double c1;
    private readonly double c2;
    private readonly double c3;

    public Gorkov(SpatialDerivatives sd0, double[] K0)
    {
        this.sd = sd0;
        this.K = K0;

        this.Uxx = 0;
        this.Uyy = 0;
        this.Uzz = 0;

        this.gUxx = 0;
        this.gUyy = 0;
        this.gUzz = 0;

        this.gpAbs = 0;
        this.pAbs = 0;

        // this.c1 = 1e-2;
        // this.c2 = 1e-4;
        // this.c3 = 1e-6;
        this.c1 = 1e-0;
        this.c2 = 1e-0;
        this.c3 = 1e-0;
    }

    public void ComputeLaplacian()
    {
        // double c1 = 1e-2;
        // double c2 = 1e-4;
        // // double c3 = 4e-4;
        // double c3 = 1e-6;
        // // double c5 = 2e-6;

        double K1 = K[0];
        double K2 = K[1];
        // Debug.Log(K1);
        // Debug.Log(K2);


        pAbs = DotOp(sd.P, sd.P);

        double Uxx_1 = 2 * K1 * (c1 * c1 * DotOp(sd.Px, sd.Px) + c1 * c2 * DotOp(sd.P, sd.Pxx));
        double Uxx_2x = K2 * (c2 * c2 * DotOp(sd.Pxx, sd.Pxx) + c1 * c3 * DotOp(sd.Px, sd.Pxxx));
        double Uxx_2y = K2 * (c2 * c2 * DotOp(sd.Pxy, sd.Pxy) + c1 * c3 * DotOp(sd.Px, sd.Pyxx));
        double Uxx_2z = K2 * (c2 * c2 * DotOp(sd.Pxz, sd.Pxz) + c1 * c3 * DotOp(sd.Px, sd.Pzxx));
        Uxx = Uxx_1 - Uxx_2x - Uxx_2y - Uxx_2z;

        double Uyy_1 = 2 * K1 * (c1 * c1 * DotOp(sd.Py, sd.Py) + c1 * c2 * DotOp(sd.P, sd.Pyy));
        double Uyy_2x = K2 * (c2 * c2 * DotOp(sd.Pxy, sd.Pxy) + c1 * c3 * DotOp(sd.Py, sd.Pxyy));
        double Uyy_2y = K2 * (c2 * c2 * DotOp(sd.Pyy, sd.Pyy) + c1 * c3 * DotOp(sd.Py, sd.Pyyy));
        double Uyy_2z = K2 * (c2 * c2 * DotOp(sd.Pyz, sd.Pyz) + c1 * c3 * DotOp(sd.Py, sd.Pzyy));
        Uyy = Uyy_1 - Uyy_2x - Uyy_2y - Uyy_2z;

        double Uzz_1 = 2 * K1 * (c1 * c1 * DotOp(sd.Pz, sd.Pz) + c1 * c2 * DotOp(sd.P, sd.Pzz));
        double Uzz_2x = K2 * (c2 * c2 * DotOp(sd.Pxz, sd.Pxz) + c1 * c3 * DotOp(sd.Pz, sd.Pxzz));
        double Uzz_2y = K2 * (c2 * c2 * DotOp(sd.Pyz, sd.Pyz) + c1 * c3 * DotOp(sd.Pz, sd.Pyzz));
        double Uzz_2z = K2 * (c2 * c2 * DotOp(sd.Pzz, sd.Pzz) + c1 * c3 * DotOp(sd.Pz, sd.Pzzz));
        Uzz = Uzz_1 - Uzz_2x - Uzz_2y - Uzz_2z;

        //

        // double Uxx_1 = 2 * K1 * (DotOp(sd.Px, sd.Px) + DotOp(sd.P, sd.Pxx));
        // double Uxx_2x = K2 * (DotOp(sd.Pxx, sd.Pxx) + DotOp(sd.Px, sd.Pxxx));
        // double Uxx_2y = K2 * (DotOp(sd.Pxy, sd.Pxy) + DotOp(sd.Px, sd.Pyxx));
        // double Uxx_2z = K2 * (DotOp(sd.Pxz, sd.Pxz) + DotOp(sd.Px, sd.Pzxx));
        // Uxx = Uxx_1 - Uxx_2x - Uxx_2y - Uxx_2z;


        // double Uyy_1 = 2 * K1 * (DotOp(sd.Py, sd.Py) + DotOp(sd.P, sd.Pyy));
        // double Uyy_2x = K2 * (DotOp(sd.Pxy, sd.Pxy) + DotOp(sd.Py, sd.Pxyy));
        // double Uyy_2y = K2 * (DotOp(sd.Pyy, sd.Pyy) + DotOp(sd.Py, sd.Pyyy));
        // double Uyy_2z = K2 * (DotOp(sd.Pyz, sd.Pyz) + DotOp(sd.Py, sd.Pzyy));
        // Uyy = Uyy_1 - Uyy_2x - Uyy_2y - Uyy_2z;


        // double Uzz_1 = 2 * K1 * (DotOp(sd.Pz, sd.Pz) + DotOp(sd.P, sd.Pzz));
        // double Uzz_2x = K2 * (DotOp(sd.Pxz, sd.Pxz) + DotOp(sd.Pz, sd.Pxzz));
        // double Uzz_2y = K2 * (DotOp(sd.Pyz, sd.Pyz) + DotOp(sd.Pz, sd.Pyzz));
        // double Uzz_2z = K2 * (DotOp(sd.Pzz, sd.Pzz) + DotOp(sd.Pz, sd.Pzzz));
        // Uzz = Uzz_1 - Uzz_2x - Uzz_2y - Uzz_2z;

        // Debug.Log(Uxx_1.ToString("F4"));
        // Debug.Log(Uxx_2x.ToString("F4"));
        // Debug.Log(Uxx_2y.ToString("F4"));
        // Debug.Log(Uxx_2z.ToString("F4"));
        // Debug.Log(Uxx.ToString("F4"));

        // Debug.Log(Uyy_1.ToString("F4"));
        // Debug.Log(Uyy_2x.ToString("F4"));
        // Debug.Log(Uyy_2y.ToString("F4"));
        // Debug.Log(Uyy_2z.ToString("F4"));
        // Debug.Log(Uyy.ToString("F4"));

        // Debug.Log(Uzz_1.ToString("F4"));
        // Debug.Log(Uzz_2x.ToString("F4"));
        // Debug.Log(Uzz_2y.ToString("F4"));
        // Debug.Log(Uzz_2z.ToString("F4"));
        // Debug.Log(Uzz.ToString("F4"));
    }

    public void ComputeGradientLaplacian(int i)
    {

        // double c1 = 1e-2;
        // double c2 = 1e-4;
        // // double c3 = 4e-4;
        // double c3 = 1e-6;
        // double c5 = 2e-6;


        double K1 = K[0];
        double K2 = K[1];

        gpAbs = D_DotOp(sd.P, sd.P, sd.p[i], sd.p[i]);

        //

        // double gUxx_1 = 2 * K1 * (  D_DotOp_14(sd.Px, sd.px[i])   + D_DotOp_23(sd.Px, sd.px[i])   + D_DotOp_14(sd.P, sd.pxx[i])   + D_DotOp_23(sd.Pxx, sd.p[i])     );
        // double gUxx_2x = K2 *    (  D_DotOp_14(sd.Pxx, sd.pxx[i]) + D_DotOp_23(sd.Pxx, sd.pxx[i]) + D_DotOp_14(sd.Px, sd.pxxx[i]) + D_DotOp_23(sd.Pxxx, sd.px[i])   );
        // double gUxx_2y = K2 *    (  D_DotOp_14(sd.Pxy, sd.pxy[i]) + D_DotOp_23(sd.Pxy, sd.pxy[i]) + D_DotOp_14(sd.Px, sd.pyxx[i]) + D_DotOp_23(sd.Pyxx, sd.px[i])   );
        // double gUxx_2z = K2 *    (  D_DotOp_14(sd.Pxz, sd.pxz[i]) + D_DotOp_23(sd.Pxz, sd.pxz[i]) + D_DotOp_14(sd.Px, sd.pzxx[i]) + D_DotOp_23(sd.Pzxx, sd.pz[i])   );
        // gUxx = gUxx_1 - gUxx_2x - gUxx_2y - gUxx_2z;

        // double gUyy_1 = 2 * K1 * (  D_DotOp_14(sd.Py, sd.py[i])   + D_DotOp_23(sd.Py, sd.py[i])   + D_DotOp_14(sd.P, sd.pyy[i])   + D_DotOp_23(sd.Pyy, sd.p[i])     );
        // double gUyy_2x = K2 *    (  D_DotOp_14(sd.Pxy, sd.pxy[i]) + D_DotOp_23(sd.Pxy, sd.pxy[i]) + D_DotOp_14(sd.Py, sd.pxyy[i]) + D_DotOp_23(sd.Pxyy, sd.py[i])   );
        // double gUyy_2y = K2 *    (  D_DotOp_14(sd.Pyy, sd.pyy[i]) + D_DotOp_23(sd.Pyy, sd.pyy[i]) + D_DotOp_14(sd.Py, sd.pyyy[i]) + D_DotOp_23(sd.Pyyy, sd.py[i])   );
        // double gUyy_2z = K2 *    (  D_DotOp_14(sd.Pyz, sd.pyz[i]) + D_DotOp_23(sd.Pyz, sd.pyz[i]) + D_DotOp_14(sd.Py, sd.pzyy[i]) + D_DotOp_23(sd.Pzyy, sd.py[i])   );
        // gUyy = gUyy_1 - gUyy_2x - gUyy_2y - gUyy_2z;

        // double gUzz_1 = 2 * K1 * (  D_DotOp_14(sd.Pz, sd.pz[i])   + D_DotOp_23(sd.Pz, sd.pz[i])   + D_DotOp_14(sd.P, sd.pzz[i])   + D_DotOp_23(sd.Pzz, sd.p[i])     );
        // double gUzz_2x = K2 *    (  D_DotOp_14(sd.Pxz, sd.pxz[i]) + D_DotOp_23(sd.Pxz, sd.pxz[i]) + D_DotOp_14(sd.Pz, sd.pxzz[i]) + D_DotOp_23(sd.Pxzz, sd.pz[i])   );
        // double gUzz_2y = K2 *    (  D_DotOp_14(sd.Pyz, sd.pyz[i]) + D_DotOp_23(sd.Pyz, sd.pyz[i]) + D_DotOp_14(sd.Pz, sd.pyzz[i]) + D_DotOp_23(sd.Pyzz, sd.pz[i])   );
        // double gUzz_2z = K2 *    (  D_DotOp_14(sd.Pzz, sd.pzz[i]) + D_DotOp_23(sd.Pzz, sd.pzz[i]) + D_DotOp_14(sd.Pz, sd.pzzz[i]) + D_DotOp_23(sd.Pzzz, sd.pz[i])   );
        // gUzz = gUzz_1 - gUzz_2x - gUzz_2y - gUzz_2z;

        //

        double gUxx_1 = 2 * K1 * (c1 * D_DotOp_14(sd.Px, sd.px[i]) + c1 * D_DotOp_23(sd.Px, sd.px[i]) + c1 * D_DotOp_14(sd.P, sd.pxx[i]) + c2 * D_DotOp_23(sd.Pxx, sd.p[i]));
        double gUxx_2x = K2 * (c2 * D_DotOp_14(sd.Pxx, sd.pxx[i]) + c2 * D_DotOp_23(sd.Pxx, sd.pxx[i]) + c1 * D_DotOp_14(sd.Px, sd.pxxx[i]) + c3 * D_DotOp_23(sd.Pxxx, sd.px[i]));
        double gUxx_2y = K2 * (c2 * D_DotOp_14(sd.Pxy, sd.pxy[i]) + c2 * D_DotOp_23(sd.Pxy, sd.pxy[i]) + c1 * D_DotOp_14(sd.Px, sd.pyxx[i]) + c3 * D_DotOp_23(sd.Pyxx, sd.px[i]));
        double gUxx_2z = K2 * (c2 * D_DotOp_14(sd.Pxz, sd.pxz[i]) + c2 * D_DotOp_23(sd.Pxz, sd.pxz[i]) + c1 * D_DotOp_14(sd.Px, sd.pzxx[i]) + c3 * D_DotOp_23(sd.Pzxx, sd.pz[i]));
        gUxx = gUxx_1 - gUxx_2x - gUxx_2y - gUxx_2z;

        double gUyy_1 = 2 * K1 * (c1 * D_DotOp_14(sd.Py, sd.py[i]) + c1 * D_DotOp_23(sd.Py, sd.py[i]) + c1 * D_DotOp_14(sd.P, sd.pyy[i]) + c2 * D_DotOp_23(sd.Pyy, sd.p[i]));
        double gUyy_2x = K2 * (c2 * D_DotOp_14(sd.Pxy, sd.pxy[i]) + c2 * D_DotOp_23(sd.Pxy, sd.pxy[i]) + c1 * D_DotOp_14(sd.Py, sd.pxyy[i]) + c3 * D_DotOp_23(sd.Pxyy, sd.py[i]));
        double gUyy_2y = K2 * (c2 * D_DotOp_14(sd.Pyy, sd.pyy[i]) + c2 * D_DotOp_23(sd.Pyy, sd.pyy[i]) + c1 * D_DotOp_14(sd.Py, sd.pyyy[i]) + c3 * D_DotOp_23(sd.Pyyy, sd.py[i]));
        double gUyy_2z = K2 * (c2 * D_DotOp_14(sd.Pyz, sd.pyz[i]) + c2 * D_DotOp_23(sd.Pyz, sd.pyz[i]) + c1 * D_DotOp_14(sd.Py, sd.pzyy[i]) + c3 * D_DotOp_23(sd.Pzyy, sd.py[i]));
        gUyy = gUyy_1 - gUyy_2x - gUyy_2y - gUyy_2z;

        double gUzz_1 = 2 * K1 * (c1 * D_DotOp_14(sd.Pz, sd.pz[i]) + c1 * D_DotOp_23(sd.Pz, sd.pz[i]) + c1 * D_DotOp_14(sd.P, sd.pzz[i]) + c1 * D_DotOp_23(sd.Pzz, sd.p[i]));
        double gUzz_2x = K2 * (c2 * D_DotOp_14(sd.Pxz, sd.pxz[i]) + c2 * D_DotOp_23(sd.Pxz, sd.pxz[i]) + c1 * D_DotOp_14(sd.Pz, sd.pxzz[i]) + c3 * D_DotOp_23(sd.Pxzz, sd.pz[i]));
        double gUzz_2y = K2 * (c2 * D_DotOp_14(sd.Pyz, sd.pyz[i]) + c2 * D_DotOp_23(sd.Pyz, sd.pyz[i]) + c1 * D_DotOp_14(sd.Pz, sd.pyzz[i]) + c3 * D_DotOp_23(sd.Pyzz, sd.pz[i]));
        double gUzz_2z = K2 * (c2 * D_DotOp_14(sd.Pzz, sd.pzz[i]) + c2 * D_DotOp_23(sd.Pzz, sd.pzz[i]) + c1 * D_DotOp_14(sd.Pz, sd.pzzz[i]) + c3 * D_DotOp_23(sd.Pzzz, sd.pz[i]));
        gUzz = gUzz_1 - gUzz_2x - gUzz_2y - gUzz_2z;

        //

        // double gUxx_1 = 2 * K1 * (D_DotOp(sd.Px, sd.Px, sd.px[i], sd.px[i]) + D_DotOp(sd.P, sd.Pxx, sd.p[i], sd.pxx[i]));
        // double gUxx_2x = K2 * (D_DotOp(sd.Pxx, sd.Pxx, sd.pxx[i], sd.pxx[i]) + D_DotOp(sd.Px, sd.Pxxx, sd.px[i], sd.pxxx[i]));
        // double gUxx_2y = K2 * (D_DotOp(sd.Pxy, sd.Pxy, sd.pxy[i], sd.pxy[i]) + D_DotOp(sd.Px, sd.Pyxx, sd.px[i], sd.pyxx[i]));
        // double gUxx_2z = K2 * (D_DotOp(sd.Pxz, sd.Pxz, sd.pxz[i], sd.pxz[i]) + D_DotOp(sd.Px, sd.Pzxx, sd.pz[i], sd.pzxx[i]));
        // gUxx = gUxx_1 - gUxx_2x - gUxx_2y - gUxx_2z;

        // double gUyy_1 = 2 * K1 * (D_DotOp(sd.Py, sd.Py, sd.py[i], sd.py[i]) + D_DotOp(sd.P, sd.Pyy, sd.p[i], sd.pyy[i]));
        // double gUyy_2x = K2 * (D_DotOp(sd.Pxy, sd.Pxy, sd.pxy[i], sd.pxy[i]) + D_DotOp(sd.Py, sd.Pxyy, sd.py[i], sd.pxyy[i]));
        // double gUyy_2y = K2 * (D_DotOp(sd.Pyy, sd.Pyy, sd.pyy[i], sd.pyy[i]) + D_DotOp(sd.Py, sd.Pyyy, sd.py[i], sd.pyyy[i]));
        // double gUyy_2z = K2 * (D_DotOp(sd.Pyz, sd.Pyz, sd.pyz[i], sd.pyz[i]) + D_DotOp(sd.Py, sd.Pzyy, sd.py[i], sd.pzyy[i]));
        // gUyy = gUyy_1 - gUyy_2x - gUyy_2y - gUyy_2z;

        // double gUzz_1 = 2 * K1 * (D_DotOp(sd.Pz, sd.Pz, sd.pz[i], sd.pz[i]) + D_DotOp(sd.P, sd.Pzz, sd.p[i], sd.pzz[i]));
        // double gUzz_2x = K2 * (D_DotOp(sd.Pxz, sd.Pxz, sd.pxz[i], sd.pxz[i]) + D_DotOp(sd.Pz, sd.Pxzz, sd.pz[i], sd.pxzz[i]));
        // double gUzz_2y = K2 * (D_DotOp(sd.Pyz, sd.Pyz, sd.pyz[i], sd.pyz[i]) + D_DotOp(sd.Pz, sd.Pyzz, sd.pz[i], sd.pyzz[i]));
        // double gUzz_2z = K2 * (D_DotOp(sd.Pzz, sd.Pzz, sd.pzz[i], sd.pzz[i]) + D_DotOp(sd.Pz, sd.Pzzz, sd.pz[i], sd.pzzz[i]));
        // gUzz = gUzz_1 - gUzz_2x - gUzz_2y - gUzz_2z;

        // Debug.Log(gUxx_1.ToString("F4"));
        // Debug.Log(gUxx_2x.ToString("F4"));
        // Debug.Log(gUxx_2y.ToString("F4"));
        // Debug.Log(gUxx_2z.ToString("F4"));
        // Debug.Log(gUxx.ToString("F4"));

        // Debug.Log(gUyy_1.ToString("F4"));
        // Debug.Log(gUyy_2x.ToString("F4"));
        // Debug.Log(gUyy_2y.ToString("F4"));
        // Debug.Log(gUyy_2z.ToString("F4"));
        // Debug.Log(gUyy.ToString("F4"));

        // Debug.Log(gUzz_1.ToString("F4"));
        // Debug.Log(gUzz_2x.ToString("F4"));
        // Debug.Log(gUzz_2y.ToString("F4"));
        // Debug.Log(gUzz_2z.ToString("F4"));
        // Debug.Log(gUzz.ToString("F4"));
    }


    private double DotOp(Complex P1, Complex P2)
    {
        return P1.Real * P2.Real + P1.Imaginary * P2.Imaginary;
    }

    private double D_DotOp(Complex P1, Complex P2, Complex P3, Complex P4)
    {
        return P1.Imaginary * P4.Real + P3.Real * P2.Imaginary
             - P1.Real * P4.Imaginary - P3.Imaginary * P2.Real;
    }

    private double D_DotOp_14(Complex P1, Complex P4)
    {
        return P1.Imaginary * P4.Real
             - P1.Real * P4.Imaginary;
    }

    private double D_DotOp_23(Complex P2, Complex P3)
    {
        return P3.Real * P2.Imaginary
             - P3.Imaginary * P2.Real;
    }



}

public class DummyParticle
{
    public Vector3 dummyPosition;

    public DummyParticle(Vector3 position)
    {
        this.dummyPosition = position;
    }

    public float GetDistanceFromX(Vector3 coord)
    {
        return Vector3.Distance(coord, this.dummyPosition);
    }

    // public float GetZAngleFromX(Vector3 coord)
    // {
    //     Vector3 vP = this.dummyPosition - coord;
    //     return Vector3.Angle(vP, Vector3.up)* Mathf.Deg2Rad;
    // }

    public double GetZAngleFromX(Vector3 pX, Vector3 vX)
    {
        Vector3 vP = this.dummyPosition - pX;
        // Debug.Log(vP);
        return Math.Acos(Vector3.Dot(vX, vP) / (Vector3.Magnitude(vX) * Vector3.Magnitude(vP)));
    }
}



public class Optimizer
{
    public Optimizer()
    {

    }

}