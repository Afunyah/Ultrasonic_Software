using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddLevParticle : MonoBehaviour
{

    public GameObject levParticlePrefab;
    public GameObject instArea;

    public void createParticle() {
        GameObject levParticle = Instantiate(levParticlePrefab, instArea.transform.position, levParticlePrefab.transform.rotation);
    }
}
