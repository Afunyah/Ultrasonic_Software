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

public class Focus : MonoBehaviour
{
    private int PHASE_RES;
    private int SOUND_SPEED;
    private int FREQ;
    private int MAX_POINTS;
    private int N_EMITTERS;
    private int ibpIterations;

    // target field at each virtual point
    // private float[] pointA;
    // private float[] pointB;
    private static Complex[] pointField;
    // complex emission of each transducer               
    // private float[] transA;
    // private float[] transB;

    private static Complex[] transducerEmission;

    // propagator from transducer to point [T][P]
    private static Complex[,] tpProp;
    // private float[,] tpB;

    void Awake()
    {
        PHASE_RES = 32;
        SOUND_SPEED = 343;
        FREQ = 40000;
        MAX_POINTS = 1;
        N_EMITTERS = 1442;
        ibpIterations = 4;

        // pointA = new float[MAX_POINTS];
        // pointB = new float[MAX_POINTS];
        // transA = new float[N_EMITTERS];
        // transB = new float[N_EMITTERS];
        // tpA = new float[N_EMITTERS, MAX_POINTS];
        // tpB = new float[N_EMITTERS, MAX_POINTS];

        pointField = new Complex[MAX_POINTS];
        transducerEmission = new Complex[N_EMITTERS];
        tpProp = new Complex[N_EMITTERS, MAX_POINTS];

    }

    void Start()
    {

    }

    void Update()
    {


    }


    /* IBP ALGORITHM - SLOWER BUT STRONGER TRAPPING FORCES */
    public void IBP_iterate(int nPoints, int nn)
    {
        // project emitters into control points
        for (int j = 0; j < nPoints; ++j)
        {
            double A = 0;
            double B = 0;
            pointField[j] = new Complex(A, B);
            for (int i = 0; i < N_EMITTERS; ++i)
            {

                A += transducerEmission[i].Real * tpProp[i, j].Real - transducerEmission[i].Imaginary * tpProp[i, j].Imaginary;
                B += transducerEmission[i].Real * tpProp[i, j].Imaginary + transducerEmission[i].Imaginary * tpProp[i, j].Real;
                pointField[j] = new Complex(A, B);
                // if(nn==1){
                //     Debug.Log(A.ToString());
                // } 
            }
        }

        // normalize control points
        for (int i = 0; i < nPoints; ++i)
        {
            float dist = (float)pointField[i].Magnitude;

            double A = pointField[i].Real / dist;
            double B = pointField[i].Imaginary / dist;

            pointField[i] = new Complex(A, B);
            Debug.Log(pointField[i].ToString());
            if (pointField[i].IsNaN())
            {
                pointField[i] = new Complex(0, 0);
            }
        }


        // backproject control points into transducers (use the conjugate to backpropagate)
        for (int i = 0; i < N_EMITTERS; ++i)
        {
            double A = 0;
            double B = 0;
            transducerEmission[i] = new Complex(A, B);
            for (int j = 0; j < nPoints; ++j)
            {
                A += pointField[j].Real * tpProp[i, j].Real - pointField[j].Imaginary * -tpProp[i, j].Imaginary;
                B += pointField[j].Real * -tpProp[i, j].Imaginary + pointField[j].Imaginary * tpProp[i, j].Real;
                transducerEmission[i] = new Complex(A, B);
                // if(nn==0){
                //     Debug.Log(transducerEmission[i].ToString());
                // }
                // Debug.Log(transducerEmission[i].ToString());
            }
        }

        // normalize transducer amplitude
        for (int i = 0; i < N_EMITTERS; ++i)
        {
            float dist = (float)transducerEmission[i].Magnitude;
            // Debug.Log(dist);
            double A = transducerEmission[i].Real / dist;
            double B = transducerEmission[i].Imaginary / dist;

            transducerEmission[i] = new Complex(A, B);

            if (transducerEmission[i].IsNaN())
            {
                transducerEmission[i] = new Complex(0, 0);
            }
        }
    }



    public Complex CalcFieldForEmitter(Vector3 emitterPos, Vector3 emitterNorm, Vector3 pointPos)
    {
        float mSpeed = SOUND_SPEED;
        float omega = (float)(2 * Math.PI * FREQ);
        float k = omega / mSpeed; // wavenumber
        k = (float)(2f * Math.PI * 343f / 40000f);
        float eApperture = 0.009f;

        // float dist = VectorDist(emitterPos, pointPos);
        // float ndist = VectorDist(emitterNorm, emitterNorm);

        Vector3 diff = pointPos - emitterPos;
        float dist = diff.magnitude;

        float ndist = emitterNorm.magnitude;

        // const static float ap = 0.009;
        // float directivity = 1; // TODO add proper directivity model

        // float ampDirAtt = directivity / dist;
        // float kdPlusPhase = k * dist + phase;

        // float fieldA = (float)(ampDirAtt * Math.Cos(kdPlusPhase));
        // float fieldB = (float)(ampDirAtt * Math.Sin(kdPlusPhase));
        double angle = Math.Acos(Vector3.Dot(diff, emitterNorm) / (ndist * dist));
        double dum = 0.5 * eApperture * k * Math.Sin(angle);
        double dire = Math.Sin(dum) / dum;
        Complex props = dire / ndist * Complex.Exp(new Complex(0, k * ndist));
        if (props.IsNaN())
        {
            return new Complex(0, 0);
        }
        return props;
    }

    public void IBP_initPropagators(int nPoints, Vector3[] emitterPos, Vector3[] emitterNorm, Vector3[] pointPos)
    {
        // initialise target points
        for (int i = 0; i < nPoints; ++i)
        {
            // we just want amplitude 1, any phase does the job to initiate them
            pointField[i] = new Complex(1, 0);

            // calculate the propagators from the transducers j into point i
            for (int j = 0; j < N_EMITTERS; ++j)
            {
                tpProp[j, i] = CalcFieldForEmitter(emitterPos[j], emitterNorm[j], pointPos[i]);
            }
        }
    }

    public void IBP_initEmitters()
    {
        for (int j = 0; j < N_EMITTERS; ++j)
        {
            transducerEmission[j] = new Complex(1, 0);
        }
    }

    public double[] IBP_applySolution()
    {
        double[] phases = new double[N_EMITTERS];
        for (int i = 0; i < N_EMITTERS; ++i)
        {
            float angle = (float)Math.Atan2(transducerEmission[i].Imaginary, transducerEmission[i].Real);
            // Debug.Log(angle);
            if (angle < 0) // while?
            {
                angle += (float)(2 * Math.PI);
            }
            // int phase = (int)(angle / (2 * Math.PI) * PHASE_RES);

            // phases[i] = phase;
            phases[i] = angle;
        }
        return phases;
    }

    public double[] FocusArrayAtPoints(Vector3[] emitterPosArray, Vector3[] emitterNormArray, Vector3[] pointPosArray)
    {
        //this is the IBP method. slow but stronger trapping points. hard to join at closer distances.
        IBP_initPropagators(1, emitterPosArray, emitterNormArray, pointPosArray);
        for (int i = 0; i < ibpIterations; ++i)
        {
            IBP_iterate(1, i);
        }

        double[] sol = IBP_applySolution();

        return sol;

    }


    public double[] SimpleFocus(List<Transducer> transducers, Vector3 pPos)
    {
        double[] phases = new double[transducers.Count];

        double wl = 343f / 40000f;
        // wl = 16.575e-3;
        wl *= 1f;

        for (int i = 0; i < transducers.Count; i++)
        {
            float dist = Vector3.Distance(SwapYZ(transducers[i].GetSolverPostion()), SwapYZ(pPos));
            double wavestep = dist / wl;
            double tphase = (1.0f - (wavestep - ((int)wavestep))) * 2.0f * Math.PI;
            phases[i] = tphase / Math.PI;
        }

        return phases;
    }

    public double[] TwinTrap(List<Transducer> transducers, double[] phases_focus, Vector3 solvrSizeVector, Vector3 solvrCenterVector, double angle)
    {
        Vector3 centerVec = SwapYZ(solvrCenterVector);
        Vector3 sizeVec = SwapYZ(solvrSizeVector);

        for (int i = 0; i < transducers.Count; i++)
        {
            Vector3 pos = SwapYZ(transducers[i].GetSolverPostion());
            Vector3 npos3 = pos - centerVec;
            npos3 = new Vector3(npos3.x / sizeVec.x, npos3.y / sizeVec.y, npos3.z / sizeVec.z);
            Vector2 p = new Vector2(npos3.x, npos3.z);

            double value = 0;
            value = (Math.Atan2(p.y, p.x) + angle) / Math.PI % 2.0f;
            if (value >= 0.0f && value <= 1.0f) { value = 0.0f; }
            else { value = 1.0f; }

            phases_focus[i] += value;
        }

        return phases_focus;
    }

    public double[] VortexTrap(List<Transducer> transducers, double[] phases_focus, Vector3 solvrSizeVector, Vector3 solvrCenterVector, double m)
    {
        Vector3 centerVec = SwapYZ(solvrCenterVector);
        Vector3 sizeVec = SwapYZ(solvrSizeVector);

        for (int i = 0; i < transducers.Count; i++)
        {
            Vector3 pos = SwapYZ(transducers[i].GetSolverPostion());
            Vector3 npos3 = pos - centerVec;
            npos3 = new Vector3(npos3.x / sizeVec.x, npos3.y / sizeVec.y, npos3.z / sizeVec.z);
            Vector2 p = new Vector2(npos3.x, npos3.z);

            double value = 0;
            value = (Math.Atan2(p.y, p.x) * m) / Math.PI % 2.0f;
            Debug.Log(value);
            phases_focus[i] += value;
        }

        return phases_focus;
    }


    public double[] VortexTrap_2(List<Transducer> transducers, double[] phases_focus, Vector3 solvrSizeVector, Vector3 solvrCenterVector, double m)
    {
        Vector3 centerVec = SwapYZ(solvrCenterVector);
        Vector3 sizeVec = SwapYZ(solvrSizeVector);

        for (int i = 0; i < transducers.Count; i++)
        {
            Vector3 pos = SwapYZ(transducers[i].GetSolverPostion());
            Vector3 npos3 = pos - centerVec;
            npos3 = new Vector3(npos3.x / sizeVec.x, npos3.y / sizeVec.y, npos3.z / sizeVec.z);
            Vector2 p2 = new Vector2(npos3.x, npos3.z);
            Vector3 p3 = new Vector3(npos3.x, npos3.y, npos3.z);

            double value = 0;
            value = m * (Math.Atan2(p2.y, p2.x) + Math.Asin(p3.y/p3.magnitude)) / Math.PI % 2.0f;
            Debug.Log(value);
            phases_focus[i] += value;
        }

        return phases_focus;
    }



    // public double[] BerritTrap_1(List<Transducer> transducers, double[] phases_focus, Vector3 solvrSizeVector, Vector3 solvrCenterVector, double m)
    // {
    //     Vector3 sizeVec = SwapYZ(solvrSizeVector);

    //     for (int i = 0; i < transducers.Count; i++)
    //     {
    //         Vector3 pos = SwapYZ(transducers[i].GetSolverPostion());
    //         Vector3 npos3 = pos - centerVec;
    //         npos3 = new Vector3(npos3.x / sizeVec.x, npos3.y / sizeVec.y, npos3.z / sizeVec.z);
    //         Vector2 p = new Vector2(npos3.x, npos3.z);

    //         Vector3 posVec_3D = tr_pos - prtc_pos;
    //         Vector2 posVec_2D;
    //         posVec_2D.x = posVec_3D.x;
    //         posVec_2D.y = posVec_3D.y;
    //         double dist = Vector3.Magnitude(posVec_3D);
    //         double azi = Math.Atan2(posVec_2D.y, posVec_2D.x);
    //         double elev = Math.Atan2(Math.Abs(posVec_3D.z), Vector3.Magnitude(posVec_2D));

    //         double value = (wavenum * dist) + m * (azi + elev);

    //         double value = 0;
    //         value = (Math.Atan2(p.y, p.x) * m) / Math.PI % 2.0f;

    //         phases_focus[i] += value;
    //     }

    //     return phases_focus;
    // }


    public double[] BerritTrap_2(List<Transducer> transducers, Vector3 prtc_pos, double m)
    {
        double[] phases = new double[transducers.Count];

        double wavelen = 343f / 40000f;

        double wavenum = 2f * Math.PI / wavelen;

        for (int i = 0; i < transducers.Count; i++)
        {
            Vector3 tr_pos = transducers[i].GetSolverPostion();

            Vector3 posVec_3D = tr_pos - prtc_pos;
            Vector2 posVec_2D;
            posVec_2D.x = posVec_3D.x;
            posVec_2D.y = posVec_3D.y;

            double dist = Vector3.Magnitude(posVec_3D);
            // double azi = Math.Atan(posVec_2D.y / posVec_2D.x);
            double azi = Math.Atan2(posVec_2D.y, posVec_2D.x);


            // double elev = Math.Atan(prtc_pos.z / Vector3.Magnitude(posVec_2D));
            double elev = Math.Atan2(Math.Abs(posVec_3D.z), Vector3.Magnitude(posVec_2D));

            double value = (wavenum * dist) + m * (azi + elev);
            // value %= 2*Math.PI;

            phases[i] = value;
        }

        return phases;
    }

    private Vector3 SwapYZ(Vector3 vec)
    {
        return new Vector3(vec.x, vec.z, vec.y);
    }


}
