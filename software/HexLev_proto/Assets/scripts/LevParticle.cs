// using System;
// using System.Collections;
// using System.Collections.Generic;
// using System.Numerics;
// using Unity.VisualScripting;
// using UnityEngine;
// using Vector3 = UnityEngine.Vector3;

// public class LevParticle : MonoBehaviour
// {
//     private Vector3 particlePos;
//     private bool selected;
//     private List<Transducer> nearbyTransducers;

//     private List<Trajectory> Trajectories;
//     private List<GameObject> Ghosts;
//     public GameObject ghostParticlePrefab;
//     private GameObject ghostParent;

//     void Start()
//     {
//         nearbyTransducers = new List<Transducer> { };
//         Trajectories = new List<Trajectory> { };
//         Ghosts = new List<GameObject> { };
//         ghostParent = new GameObject("ghostParent");
//         particlePos = this.transform.position;
//         selected = false;
//     }

//     void Update()
//     {
//         particlePos = this.transform.position;
//         foreach (Transducer x in nearbyTransducers)
//         {
//             // Debug.Log(x.name);
//         }

//         foreach (Trajectory x in Trajectories)
//         {
//             Debug.DrawLine(x.GetStartPoint(), x.GetEndPoint(), Color.red);
//         }

//     }

//     public List<Transducer> findNearbyTransducers()
//     {
//         return nearbyTransducers;
//     }

//     private void OnTriggerStay(Collider other)
//     {
//         // Debug.Log(other.gameObject.name);
//         Transducer trs = other.gameObject.GetComponent<Transducer>();

//         if (trs != null && !trs.IsActive())
//         {
//             trs.Activate();
//             nearbyTransducers.Add(trs);
//         }
//     }

//     private void OnTriggerExit(Collider other)
//     {
//         // Debug.Log(other.gameObject.name);
//         Transducer trs = other.gameObject.GetComponent<Transducer>();

//         if (trs != null)
//         {
//             trs.Deactivate();
//             nearbyTransducers.Remove(trs);
//         }
//     }

//     public void MoveX(int dir)
//     {
//         this.transform.position += new Vector3(dir * 0.1F, 0, 0);
//     }

//     public void MoveY(int dir)
//     {
//         this.transform.position += new Vector3(0, 0, dir * 0.1F);
//     }

//     public void MoveZ(int dir)
//     {
//         this.transform.position += new Vector3(0, dir * 0.1F, 0);
//     }

//     public void SetSelect(bool sel)
//     {
//         selected = sel;
//     }

//     public void DeleteParticle()
//     {
//         foreach (Transform c in this.ghostParent.transform)
//         {
//             GameObject.Destroy(c.gameObject);
//         }
//         GameObject.Destroy(this.ghostParent);
//         Destroy(gameObject);
//     }

//     public Vector3 GetPosition()
//     {
//         return particlePos;
//     }

//     public void AddTrajectory(Vector3 A, Vector3 B)
//     {
//         if (Trajectories.Count != 0 && Trajectories[^1].GetEndPoint() != A)
//         {
//             return;
//         }
//         Trajectory traj = new Trajectory(A, B, 0.1f);
//         Trajectories.Add(traj);
//         this.AddGhostParticles();
//     }

//     public void AddGhostParticles()
//     {
//         foreach (Transform c in this.ghostParent.transform)
//         {
//             GameObject.Destroy(c.gameObject);
//         }
//         GameObject.Destroy(this.ghostParent);

//         ghostParent = new GameObject("ghostParent");
//         ghostParent.transform.position = this.Trajectories[0].GetStartPoint();
//         foreach (Trajectory x in this.Trajectories)
//         {

//             List<Vector3> tpath = new List<Vector3> { };
//             tpath = x.GetPath();
//             Debug.Log(tpath.Count);
//             foreach (Vector3 y in tpath)
//             {
//                 GameObject ghostParticle = Instantiate(ghostParticlePrefab, y, this.transform.rotation);
//                 Ghosts.Add(ghostParticle);
//                 ghostParticle.transform.SetParent(this.ghostParent.transform);
//             }

//         }

//     }
// }


// public class Trajectory
// {
//     private readonly Vector3 StartPoint;
//     private readonly Vector3 EndPoint;
//     private readonly List<Vector3> tPath;
//     private readonly float Res;
//     public Trajectory(Vector3 A, Vector3 B, float res)
//     {
//         StartPoint = A;
//         EndPoint = B;
//         Res = res;
//         tPath = this.CalculatePath();
//     }

//     public Vector3 GetStartPoint()
//     {
//         return this.StartPoint;
//     }

//     public Vector3 GetEndPoint()
//     {
//         return this.EndPoint;
//     }
//     public List<Vector3> GetPath()
//     {
//         return tPath;
//     }

//     public List<Vector3> CalculatePath()
//     {
//         List<Vector3> p = new List<Vector3> { };
//         Vector3 point = this.GetStartPoint();
//         while (Vector3.Distance(point, this.GetEndPoint()) > 0.001f)
//         {
//             // Debug.Log(point);
//             p.Add(point);
//             point = Vector3.MoveTowards(point, this.GetEndPoint(), this.Res);
//         }
//         return p;
//     }

// }




using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class LevParticle : MonoBehaviour
{
    private Vector3 particlePos;
    private bool selected;

    private List<Trajectory> Trajectories;
    private List<GhostParticle> GhostParticles;
    public GameObject ghostParticlePrefab;
    private GameObject ghostParent;

    void Start()
    {
        Trajectories = new List<Trajectory> { };
        GhostParticles = new List<GhostParticle> { };
        ghostParent = new GameObject("ghostParent");
        particlePos = this.transform.position;
        selected = false;
    }

    void Update()
    {
        particlePos = this.transform.position;

        foreach (Trajectory x in Trajectories)
        {
            Debug.DrawLine(x.GetStartPoint(), x.GetEndPoint(), Color.red);
        }

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

    public void SetSelect(bool sel)
    {
        selected = sel;
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

    public Vector3 GetPosition()
    {
        return particlePos;
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

    public void AddGhostParticles()
    {
        foreach (Transform c in this.ghostParent.transform)
        {
            GameObject.Destroy(c.gameObject);
        }
        GameObject.Destroy(this.ghostParent);

        ghostParent = new GameObject("ghostParent");
        ghostParent.transform.position = this.Trajectories[0].GetStartPoint();
        foreach (Trajectory traj in this.Trajectories)
        {
            List<Vector3> tpath = new List<Vector3> { };
            tpath = traj.GetPath();
            foreach (Vector3 point in tpath)
            {
                GameObject ghost = Instantiate(ghostParticlePrefab, point, this.transform.rotation);
                ghost.transform.SetParent(this.ghostParent.transform);

                GhostParticles.Add(ghost.GetComponent<GhostParticle>());
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


