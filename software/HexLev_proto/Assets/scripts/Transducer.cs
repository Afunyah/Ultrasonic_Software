using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Transducer : MonoBehaviour
{
    private Vector3 TPosition;
    // Start is called before the first frame update
    void Start()
    {
        TPosition = this.gameObject.transform.localPosition + this.gameObject.transform.parent.transform.localPosition;
        BoxCollider Bcollider = this.AddComponent<BoxCollider>();
        // Bcollider.center = this.transform.position;
        Bcollider.size = new Vector3(0.006858086F,0.006858086F,0.01109307F);
        // Debug.Log(TPosition);
    }

    // Update is called once per frame
    void Update()
    {
        Debug.Log(this.transform.position);
    }
}
