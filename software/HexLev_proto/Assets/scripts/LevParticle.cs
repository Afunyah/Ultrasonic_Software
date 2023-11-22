using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;
using Vector2 = UnityEngine.Vector2;

public class LevParticle : MonoBehaviour
{
    private Vector3 particlePos;
    private bool selected;

    private List<Trajectory> Trajectories;
    public GameObject ghostParticlePrefab;
    private GameObject ghostParent;

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

        foreach (Trajectory x in Trajectories)
        {
            Debug.DrawLine(x.GetStartPoint(), x.GetEndPoint(), Color.red);
        }

    }

    public Vector3 GetPosition()
    {
        return particlePos;
    }

    public void SetSelect(bool sel)
    {
        selected = sel;
    }

    public void MoveX(int dir)
    {
        this.transform.position += new Vector3(dir * 0.12F, 0, 0);
    }

    public void MoveY(int dir)
    {
        this.transform.position += new Vector3(0, 0, dir * 0.12F);
    }

    public void MoveZ(int dir)
    {
        this.transform.position += new Vector3(0, dir * 0.12F, 0);
    }

    public void DeleteParticle()
    {
        foreach (Transform c in this.ghostParent.transform)
        {
            GameObject.Destroy(c.gameObject);
        }
        GameObject.Destroy(this.ghostParent);
        Destroy(gameObject);
    }



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

    public List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> GetFullTrajectoryTransducerDataList()
    {
        return this.FullTrajectoryTransducerDataList;
    }
}



public class Trajectory
{
    private readonly Vector3 StartPoint;
    private readonly Vector3 EndPoint;
    private readonly List<Vector3> tPath;
    private readonly float Res;
    private List<GhostParticle> GhostParticles;
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

    public Vector3 GetStartPoint()
    {
        return this.StartPoint;
    }

    public Vector3 GetEndPoint()
    {
        return this.EndPoint;
    }
    public List<Vector3> GetPath()
    {
        return tPath;
    }

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

    public void AddGhostParticle(GhostParticle gst)
    {
        this.GhostParticles.Add(gst);
    }

    public List<GhostParticle> GetGhostParticles()
    {
        return this.GhostParticles;
    }

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

            next_transducers_filtered = next_transducers.Except(transducers).ToList();

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

            this.TrajectoryTransducerData.Add((p2t1List, p2t2List));
        }
    }

    private void StandardiseTrajectoryTransducerData(List<GhostTransducerPositionData> gtpd_list, float d_max)
    {
        foreach (GhostTransducerPositionData gtpd in gtpd_list)
        {
            gtpd.DivideDist(d_max);
        }
    }

    public List<(List<GhostTransducerPositionData>, List<GhostTransducerPositionData>)> GetTrajectoryTransducerData()
    {
        if (this.TrajectoryTransducerData.Count == 0)
        {
            this.CalculateTrajectoryTransducerData();
        }
        return this.TrajectoryTransducerData;
    }

}


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

    public float GetDist()
    {
        return this.dist;
    }
    public void DivideDist(float d)
    {
        this.dist /= d;
    }
}


