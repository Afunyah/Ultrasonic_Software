using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MenuHandler : MonoBehaviour
{
    
    public void addLevParticle(){
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = new Vector3(0, 1.5f, 0);
        sphere.AddComponent<LevParticle>();
        Debug.Log("Particle Added");
    }

    public void removeLevParticle(){
        Debug.Log("Particle Removed");
    }
}
