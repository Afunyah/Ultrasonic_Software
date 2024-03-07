using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Renders pre-game, allowing a Unity Prefab and an instantiation area for a LevParticle to be specified.
/// The input GameObjects are provided pre-build within the Unity editor.
/// </summary>
public class AddLevParticle : MonoBehaviour
{

    /// <summary>
    /// The GameObject to be used for the LevParticle. Provided pre-build as a prefab.
    /// </summary>
    public GameObject levParticlePrefab;

    /// <summary>
    /// The GameObject to locate the instantiation area. Provided as an empty axis anywhere within the space, ideally close to the levitator. 
    /// </summary>
    public GameObject instArea; 
    
    /// <summary>
    /// Creates a new GameObject with the levParticlePrefab in the instArea.
    /// </summary>
    public void CreateParticle()
    {
        GameObject levParticle = Instantiate(levParticlePrefab, instArea.transform.position, levParticlePrefab.transform.rotation);
    }
}
