using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

/// <summary>
/// Class for storing transducer data
/// </summary>
public class Transducer : MonoBehaviour
{
    private Vector3 tPosition;
    private int tIndex;
    private int tArr;
    private int tPhase;
    private int tAmplitude;

    private bool used;

    void Start()
    {
    }

    void Update()
    {
    }

/// <summary>
/// Initialises the transducer by assigning an array (plate) and index to it.
/// </summary>
/// <param name="arr"> The parent plate, either top or bottom</param>
/// <param name="ind" The index of the transducer, to match any physical implementation></param>
/// <param name="bCenter_z"> Specifies the center of the transducer collider </param>
    public void Init(int arr, int ind, float bCenter_z)
    {
        used = false;
        tPhase = 0;
        tAmplitude = 0;
        tPosition = this.transform.position;
        tArr = arr;
        tIndex = ind;
        float bOff = tArr == 0 ? 1F : -1F;
        if (arr == 1 && ind == 1)
        {
        }

        // Create transducer collider
        BoxCollider Bcollider = this.AddComponent<BoxCollider>();
        // Bcollider.center = new Vector3(0, 0, (bCenter_z-this.gameObject.transform.position.y)/10-this.gameObject.transform.localPosition.z-bOff*((bCenter_z-this.gameObject.transform.position.y)/10));

        // Bcollider.center = new Vector3(0, 0, ((bCenter_z - this.gameObject.transform.position.y) / 10 - (this.gameObject.transform.localPosition.z)) + 0);
        Bcollider.center = new Vector3(0, 0, ((bCenter_z) / 14.5813F) * bOff - bOff * 0.00201F);

        Bcollider.size = new Vector3(0.006858086F, 0.006858086F, ((bCenter_z) / 10) * 2 - 2 * 0.04F - 2 * 0.00201F);
        this.gameObject.layer = LayerMask.NameToLayer("TransducerLayer");
    }
/// <summary>
/// Turns the transducer on
/// </summary>
    public void Activate()
    {
        used = true;
    }

/// <summary>
/// Turns the transducer off
/// </summary>
    public void Deactivate()
    {
        used = false;
    }

/// <summary>
/// Gets the state of the transducer; on or off
/// </summary>
/// <returns>True if transducer is on</returns>
    public bool IsActive()
    {
        return used;
    }

/// <summary>
/// Gets the XYZ postion of the transducer
/// </summary>
/// <returns>3D position vector</returns>
    public Vector3 GetPosition()
    {
        return tPosition;
    }

/// <summary>
/// Gets the XY postion of the transducer rounded to the nearest integer
/// </summary>
/// <returns>2D position vector</returns>
    public Vector2 GetXYPositionRounded()
    {
        return new Vector2(Mathf.Round(this.GetPosition().x * 100) / 100, Mathf.Round(this.GetPosition().z * 100) / 100);
    }

/// <summary>
/// Gets the XY postion of the transducer
/// </summary>
/// <returns>2D position vector</returns>
    public Vector2 GetXYPosition()
    {
        return new Vector2(this.GetPosition().x, this.GetPosition().z);
    }

/// <summary>
/// Returns the phase of the transducer
/// </summary>
/// <returns>Currently, Phase as a percentage</returns>
    public int GetPhase()
    {
        return this.tPhase;
    }

    /// <summary>
    /// Returns the amplitude of the transducer
    /// </summary>
    /// <returns>Currently, Amplitude as a percentage</returns>
    public int GetAmplitude()
    {
        return this.tAmplitude;
    }

    /// <summary>
    /// Sets the phase of the transducer
    /// </summary>
    /// <param name="p">Phase (currently 0%-100%)</param>
    public void SetPhase(int p)
    {
        this.tPhase = p;
    }

    /// <summary>
    /// Sets the amplitude of the transducer
    /// </summary>
    /// <param name="a">Amplitude (currently 0%-100%)</param>
    public void SetAmplitude(int a)
    {
        this.tAmplitude = a;
    }

}
