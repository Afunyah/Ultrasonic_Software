using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEngine;
using Vector2 = UnityEngine.Vector2;
using Vector3 = UnityEngine.Vector3;

/// <summary>
/// A ghost particle for keeping track of a LevParticle's trajectory.
/// Faded and slighlty transparent partices placed along points of a trajectory to indicate future positions of particles.
/// Useful in checking for path collisions and visualising the workspace.
/// </summary>
public class GhostParticle : MonoBehaviour
{
    /// <summary>
    /// Stores the coordinates of the ghost.
    /// </summary>
    private Vector3 particlePos;

    /// <summary>
    /// Stores a list of nearby Transducers to be used in controlling the movement of a LevParticle when it reaches this GhostParticle.
    /// </summary>
    private List<Transducer> NearbyTransducers;

    /// <summary>
    /// Specifies the radius of the sphere collider used to determine nearby Transducers. Provided by the SphereCollider in the Unity editor.
    /// </summary>
    private float ColliderRadius;

    void Awake()
    {
        NearbyTransducers = new List<Transducer>();
        ColliderRadius = this.GetComponent<SphereCollider>().radius * this.transform.localScale.x;
        particlePos = this.transform.position;
        this.gameObject.layer = LayerMask.NameToLayer("TransducerLayer"); // Specify transducer layer
    }

    void OnValidate()
    {
        particlePos = this.transform.position;
    }

    void Start()
    {

    }


    void Update()
    {
        foreach (Transducer x in NearbyTransducers)
        {
            // Debug.Log(x.name);
        }
    }

    /// <summary>
    /// Gets the position of the ghost particle
    /// </summary>
    /// <returns>Position of the ghost particle</returns>
    public Vector3 GetPostion()
    {
        return this.particlePos;
    }

    /// <summary>
    /// Gets the rounded 2D coordinates of the ghost particle.
    /// The z-coordinate is used as the y-coordinate, as in the Unity space.
    /// </summary>
    /// <returns>(x,y) rounded vector coordinates of the ghost particle</returns>
    public Vector2 GetXYPositionRounded()
    {
        return new Vector2(Mathf.Round(this.particlePos.x * 100) / 100, Mathf.Round(this.particlePos.z * 100) / 100);
    }

    /// <summary>
    /// Gets the unrounded 2D coordinates of the ghost particle.
    /// The z-coordinate is used as the y-coordinate, as in the Unity space.
    /// </summary>
    /// <returns>(x,y) vector coordinates of the ghost particle</returns>
    public Vector2 GetXYPosition()
    {
        return new Vector2(this.particlePos.x, this.particlePos.z);
    }

    /// <summary>
    /// Finds Transducers in the vicinity of the ghost particle.
    /// The position of transducers and ghost particles are projected onto a 2D plane. 
    /// This is done by checking collisions between the SphereCollider and elongated colliders of the Transducers.
    /// The closeness is defined by the SphereCollider radius.
    /// </summary>
    /// <returns>A list of nearby Transducer objects</returns>
    public List<Transducer> FindNearbyTransducers()
    {
        Collider[] forcedCollisions = Physics.OverlapSphere(this.transform.position, ColliderRadius);
        foreach (Collider hitColliders in forcedCollisions)
        {
            // this.gameObject.SendMessage("OnTriggerStay", hitColliders);
            if (hitColliders.GetComponent<Transducer>() != null)
            {
                Transducer trs = hitColliders.GetComponent<Transducer>();
                this.NearbyTransducers.Add(trs);
            }
        }


        return this.NearbyTransducers;
    }
}
