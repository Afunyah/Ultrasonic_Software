using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;


/// <summary>
/// A LevParticle is a particle to be levitated within the space.
/// This class is attached to a particle by AddLevParticle, which is controlled by the FlowHandler.
/// All the information concerning a particle is stored in this class; trajectory, ghosts, postion, status etc.
/// </summary>
public class LevParticle : MonoBehaviour
{
    /// <summary>
    /// Stores the position of the particle
    /// </summary>
    private Vector3 particlePos;

    private Vector3 dummySolverPosition;

    /// <summary>
    /// Keeps track of whether the particle is selected or not 
    /// </summary>
    private bool selected;

    /// <summary>
    /// Stores a list of Trajectory objects. Multiple trajectories can be chained.
    /// </summary>
    private List<Trajectory> Trajectories;

    /// <summary>
    /// Prefab indicating the GhostParticles along trajectories. Provided within the Unity editor.
    /// </summary>
    public GameObject ghostParticlePrefab;

    /// <summary>
    /// Gameobject which holds the GhostParticles of the particle. Each LevParticle keeps track of its ghosts.
    /// </summary>
    private GameObject ghostParent;

    /// <summary>
    /// Stores a list of tuples which contains a pair of data. 
    /// The first element of the tuple describes the relationship between a target position and a current transducer.
    /// The second element of the tuple describes the relationship between a target position and the next transducer.
    /// The list of these tuples describes the full trajectory in terms of target positions and involved transducers, step-by-step.
    /// </summary>
    private List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> FullTrajectoryTransducerDataList;

    void Awake()
    {
        Trajectories = new List<Trajectory> { };
        ghostParent = new GameObject("ghostParent");
        particlePos = this.transform.position;
        selected = false;
        FullTrajectoryTransducerDataList = new List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> { };
    }

    void Start()
    {
    }

    void Update()
    {
        particlePos = this.transform.position;

        // Draw debug line as trajectory path
        foreach (Trajectory x in Trajectories)
        {
            Debug.DrawLine(x.GetStartPoint(), x.GetEndPoint(), Color.red);
        }

    }

    /// <summary>
    /// Gets the postion of the particle.
    /// </summary>
    /// <returns>3D coordinates of the particle</returns>
    public Vector3 GetPosition()
    {
        return particlePos;
    }

    /// <summary>
    /// Sets the select status of the particle.
    /// </summary>
    /// <param name="sel">The particle is selected?</param>
    public void SetSelect(bool sel)
    {
        selected = sel;
    }

    /// <summary>
    /// Moves the particle in the x-direction.
    /// </summary>
    /// <param name="dir">Specifies the direction. -1 indicates backwards and 1 indicates forward.</param>
    public void MoveX(int dir)
    {
        this.transform.position += new Vector3(dir * 0.06F, 0, 0);
    }

    /// <summary>
    /// Moves the particle in the y-direction.
    /// </summary>
    /// <param name="dir">Specifies the direction. -1 indicates backwards and 1 indicates forward.</param>
    public void MoveY(int dir)
    {
        this.transform.position += new Vector3(0, 0, dir * 0.06F);
    }

    /// <summary>
    /// Moves the particle in the z-direction.
    /// </summary>
    /// <param name="dir">Specifies the direction. -1 indicates backwards and 1 indicates forward.</param>
    public void MoveZ(int dir)
    {
        this.transform.position += new Vector3(0, dir * 0.06F, 0);
    }



    /// <summary>
    /// Moves the particle in the z-direction.
    /// </summary>
    /// <param name="dir">Specifies the direction. -1 indicates backwards and 1 indicates forward.</param>
    public float GetDistanceFromX(Vector3 coord)
    {
        return Vector3.Distance(coord, this.GetPosition());
    }

    public float GetZAngleFromX(Vector3 coord){
        Vector3 vP = this.GetPosition() - coord;
        return Vector3.Angle(vP, Vector3.up);
    }

    public Vector3 GetSolverPosition(float scale) { 
        return this.transform.position/scale;
    }



    /// <summary>
    /// Deletes the particle. This also removes the ghost particles.
    /// </summary>
    public void DeleteParticle()
    {
        foreach (Transform c in this.ghostParent.transform)
        {
            GameObject.Destroy(c.gameObject);
        }
        GameObject.Destroy(this.ghostParent);
        Destroy(gameObject);
    }

    public void SetDummySolverPosition(Vector3 pos){
        this.dummySolverPosition = pos;
    }

    public Vector3 GetDummySolverPosition(){
        return this.dummySolverPosition;
    }


    /// <summary>
    /// Creates and adds a new Trajectory to the list of the particle's Trajectory.
    /// The startpoint of the trajectory must be the same as the endpoint of the previous trajectory (if it exists).
    /// </summary>
    /// <param name="A">3D-coordinates indicating the start point of the trajectory</param>
    /// <param name="B">3D-coordinates indicating the end point of the trajectory</param>
    public void AddTrajectory(Vector3 A, Vector3 B)
    {
        if (Trajectories.Count != 0 && Trajectories[^1].GetEndPoint() != A)
        {
            return;
        }
        Trajectory traj = new Trajectory(A, B, 0.06f);
        Trajectories.Add(traj);
        this.CreateGhostParticles();
    }

    /// <summary>
    /// Creates GhostParticles along the particle's trajectory.
    /// The ghost parent is deleted and recreated. To this effect, all the previous ghost particles are deleted and created from the trajectories list.
    /// This function is called by the AddTrajectory function.
    /// </summary>
    private void CreateGhostParticles()
    {
        foreach (Transform c in this.ghostParent.transform)
        {
            GameObject.Destroy(c.gameObject);
        }
        GameObject.Destroy(this.ghostParent);

        this.ghostParent = new GameObject("ghostParent");
        this.ghostParent.transform.position = this.Trajectories[0].GetStartPoint();
        foreach (Trajectory traj in this.Trajectories)
        {
            List<Vector3> tpath = traj.GetPath();
            foreach (Vector3 point in tpath)
            {
                GameObject ghost = Instantiate(ghostParticlePrefab, point, this.transform.rotation);
                ghost.transform.SetParent(this.ghostParent.transform);

                traj.AddGhostParticle(ghost.GetComponent<GhostParticle>());
            }
            this.FullTrajectoryTransducerDataList.AddRange(traj.GetTrajectoryTransducerData());
        }
    }

    /// <summary>
    /// Gets the full trajectory-transducer data list
    /// </summary>
    /// <returns>List of the trajectory-transducer data list</returns>
    public List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> GetFullTrajectoryTransducerDataList()
    {
        return this.FullTrajectoryTransducerDataList;
    }
}




/// <summary>
/// This class contains Trajectory data related to a given LevParticle.
/// </summary>
public class Trajectory
{
    /// <summary>
    /// 3D-coordinates specifying the start point of the trajectory.
    /// </summary>
    private readonly Vector3 StartPoint;

    /// <summary>
    /// 3D-coordinates specifying the end point of the trajectory.
    /// </summary>
    private readonly Vector3 EndPoint;

    /// <summary>
    /// A list of 3D-coordinates (points) connecting the start point to the end point.
    /// </summary>
    private readonly List<Vector3> tPath;

    /// <summary>
    /// The resolution of the trajectory in Unity units. Controls the frequency of intermediate points.
    /// </summary>
    private readonly float Res;

    /// <summary>
    /// Stores the list of GhostParticles along the trajectory.
    /// </summary>
    private List<GhostParticle> GhostParticles;

    /// <summary>
    /// Stores the list of data relating the GhostParticles and Transducers along the trajectory.
    /// </summary>
    private List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> TrajectoryTransducerData;
    public Trajectory(Vector3 A, Vector3 B, float res)
    {
        StartPoint = A;
        EndPoint = B;
        Res = res;
        tPath = this.CalculatePath();
        GhostParticles = new List<GhostParticle> { };
        TrajectoryTransducerData = new List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> { };
    }


    /// <summary>
    /// Gets the start point of the trajectory
    /// </summary>
    /// <returns>3D-Coordinates of the start point</returns>
    public Vector3 GetStartPoint()
    {
        return this.StartPoint;
    }

    /// <summary>
    /// Gets the end point of the trajectory
    /// </summary>
    /// <returns>3D-Coordinates of the end point</returns>
    public Vector3 GetEndPoint()
    {
        return this.EndPoint;
    }

    /// <summary>
    /// Gets the path of the trajectory.
    /// </summary>
    /// <returns>List containing 3D-Coordinates of all points along the trajectory</returns>
    public List<Vector3> GetPath()
    {
        return tPath;
    }

    /// <summary>
    /// Calculate the path using the resolution of the trajectory. Places points inbetween the start and end points.
    /// </summary>
    /// <returns>List of all the points along the trajectory</returns>
    public List<Vector3> CalculatePath()
    {
        List<Vector3> p = new List<Vector3> { };
        Vector3 point = this.GetStartPoint();
        while (Vector3.Distance(point, this.GetEndPoint()) > 0.001f)
        {
            p.Add(point);
            point = Vector3.MoveTowards(point, this.GetEndPoint(), this.Res);
        }
        p.Add(this.GetEndPoint());
        return p;
    }

    /// <summary>
    /// Adds a GhostParticle along the trajectory
    /// </summary>
    /// <param name="gst">GhostParticle to be added</param>
    public void AddGhostParticle(GhostParticle gst)
    {
        this.GhostParticles.Add(gst);
    }

    /// <summary>
    /// Gets the list of GhostParticles for the trajectory.
    /// </summary>
    /// <returns>List of GhostParticles</returns>
    public List<GhostParticle> GetGhostParticles()
    {
        return this.GhostParticles;
    }

    /// <summary>
    /// From the nearby transducers on a given path, calculate near and far transducers to be used by the chosen phase and amplitude manipulation method.
    /// This takes into account two ghost particles in sequence, allowing for the future moves of the particle to be predicted
    /// The transducers involved for each ghost particle are separated, with the intersecting transducers being allocated to the nearer ghost
    /// Adds a tuple of lists to the TrajectoryTransducerData
    /// The first element contains GhostTransducerPositionData pertaining to 'Postion 2, Transducer 1'
    /// The second element contains GhostTransducerPositionData pertaining to 'Postion 2, Transducer 2'
    /// </summary>
    private void CalculateTrajectoryTransducerData()
    {
        GhostParticle ghost;
        GhostParticle next_ghost;
        List<Transducer> transducers = new List<Transducer> { };
        List<Transducer> next_transducers = new List<Transducer> { };
        List<Transducer> next_transducers_filtered = new List<Transducer> { };

        List<GhostTransducerPositionData> p2t1List;
        List<GhostTransducerPositionData> p2t2List;

        for (int i = 0; i < this.GhostParticles.Count - 1; i++)
        {
            ghost = GhostParticles[i];
            next_ghost = GhostParticles[i + 1];
            transducers = ghost.FindNearbyTransducers();
            next_transducers = next_ghost.FindNearbyTransducers();

            // Fiter intersecting transducers
            next_transducers_filtered = next_transducers.Except(transducers).ToList();

            // USEFUL FOR DEBUGGING
            p2t1List = new List<GhostTransducerPositionData> { };
            p2t2List = new List<GhostTransducerPositionData> { };
            float p2t1max = 0.0f;
            float p2t2max = 0.0f;
            foreach (Transducer trs in transducers)
            {
                GhostTransducerPositionData gtpd = new GhostTransducerPositionData(trs, ghost, next_ghost);
                p2t1List.Add(gtpd);
                p2t1max = gtpd.GetDist() > p2t1max ? gtpd.GetDist() : p2t1max;
            }
            foreach (Transducer trs in next_transducers_filtered)
            {
                GhostTransducerPositionData gtpd = new GhostTransducerPositionData(trs, ghost, next_ghost);
                p2t2List.Add(gtpd);
                p2t2max = gtpd.GetDist() > p2t2max ? gtpd.GetDist() : p2t2max;
            }
            // this.StandardiseTrajectoryTransducerData(p2t1List, p2t1max);
            // this.StandardiseTrajectoryTransducerData(p2t2List, p2t2max);
            // END DEBUGGING

            this.TrajectoryTransducerData.Add((p2t1List, p2t2List));
        }
    }

    /// <summary>
    /// Standardises (min-max) the distances in trajectory data. Useful for debugging purposes
    /// </summary>
    /// <param name="gtpd_list">GhostTransducerPositionData list on trajectory</param>
    /// <param name="d_max">Maximum distance on trajectory</param>
    private void StandardiseTrajectoryTransducerData(List<GhostTransducerPositionData> gtpd_list, float d_max)
    {
        foreach (GhostTransducerPositionData gtpd in gtpd_list)
        {
            gtpd.DivideDist(d_max);
        }
    }

    /// <summary>
    /// Returns data which relates points on a trajectory to transducers surrounding the trajectory.
    /// See CalculateTrajectoryTransducerData().
    /// </summary>
    /// <returns>A list of tuples containing a pair of lists (GhostTransducerPositionData)</returns>
    public List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> GetTrajectoryTransducerData()
    {
        if (this.TrajectoryTransducerData.Count == 0)
        {
            this.CalculateTrajectoryTransducerData();
        }
        return this.TrajectoryTransducerData;
    }

}



/// <summary>
/// This class contains data for a given transducer along a path and the corresponding 'previous' and 'next' ghost particles
/// The angle of exit/approach and magnitude stored in this class is used to determine the amplitude and phase control of surrounding transducers
/// </summary>
public class GhostTransducerPositionData
{
    private float dist;
    public readonly float ang;

    public readonly Transducer trs;
    public readonly GhostParticle gst1;
    public readonly GhostParticle gst2;

    public GhostTransducerPositionData(Transducer tr, GhostParticle gs1, GhostParticle gs2)
    {
        this.trs = tr;
        this.gst1 = gs1;
        this.gst2 = gs2;
        Vector2 trs_xy = trs.GetXYPositionRounded();
        Vector2 gst1_xy = gst1.GetXYPositionRounded();
        Vector2 gst2_xy = gst2.GetXYPositionRounded();

        this.dist = (gst2.GetXYPosition() - trs.GetXYPosition()).magnitude;
        this.ang = Vector3.SignedAngle((gst1_xy - gst2_xy), (gst2_xy - trs_xy), Vector3.back);
    }

    /// <summary>
    /// Returns the 'next' distance.
    /// </summary>
    /// <returns>The distance beween the transducer and the next ghost particle</returns>
    public float GetDist()
    {
        return this.dist;
    }
    public void DivideDist(float d)
    {
        this.dist /= d;
    }
}


