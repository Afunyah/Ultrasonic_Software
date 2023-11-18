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
    private List<GhostParticle> GhostParticles;
    public GameObject ghostParticlePrefab;
    private GameObject ghostParent;

    private bool isframe;

    void Awake()
    {
        Trajectories = new List<Trajectory> { };
        GhostParticles = new List<GhostParticle> { };
        ghostParent = new GameObject("ghostParent");
        particlePos = this.transform.position;
        selected = false;
    }

    void Start()
    {
        isframe = false;
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
        this.transform.position += new Vector3(dir * 0.1F, 0, 0);
    }

    public void MoveY(int dir)
    {
        this.transform.position += new Vector3(0, 0, dir * 0.1F);
    }

    public void MoveZ(int dir)
    {
        this.transform.position += new Vector3(0, dir * 0.1F, 0);
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
        Trajectory traj = new Trajectory(A, B, 0.1f);
        Trajectories.Add(traj);
        this.AddGhostParticles();
    }

    private void AddGhostParticles()
    {
        foreach (Transform c in this.ghostParent.transform)
        {
            GameObject.Destroy(c.gameObject);
        }
        GameObject.Destroy(this.ghostParent);

        this.ghostParent = new GameObject("ghostParent");
        this.ghostParent.transform.position = this.Trajectories[0].GetStartPoint();
        this.GhostParticles = new List<GhostParticle> { };
        foreach (Trajectory traj in this.Trajectories)
        {
            List<Vector3> tpath = traj.GetPath();
            foreach (Vector3 point in tpath)
            {
                GameObject ghost = Instantiate(ghostParticlePrefab, point, this.transform.rotation);
                ghost.transform.SetParent(this.ghostParent.transform);

                this.GhostParticles.Add(ghost.GetComponent<GhostParticle>());
            }
        }
        this.Something();
    }

    private void Something()
    {
        List<Transducer> transducers = new List<Transducer> { };


        for (int i = 0; i < GhostParticles.Count - 1; i++)
        {
            GhostParticle ghost = GhostParticles[i];
            GhostParticle next_ghost = GhostParticles[i + 1];
            transducers = ghost.FindNearbyTransducers();
            List<Transducer> sortedTransducers = transducers.OrderBy(x => Vector2.Distance(x.GetXYPosition(), next_ghost.GetXYPosition())).ToList();

            if (i == 1)
            {
                foreach (Transducer trs in sortedTransducers)
                {
                    Debug.Log(trs.gameObject.name);
                }
            }
        }
    }
}



public class Trajectory
{
    private readonly Vector3 StartPoint;
    private readonly Vector3 EndPoint;
    private readonly List<Vector3> tPath;
    private readonly float Res;
    public Trajectory(Vector3 A, Vector3 B, float res)
    {
        StartPoint = A;
        EndPoint = B;
        Res = res;
        tPath = this.CalculatePath();
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

}


