using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetAllShaders : MonoBehaviour
{

    [SerializeField]
    private Material material;
    MeshRenderer[] renderers;


    void Awake()
    {
        renderers = GetComponentsInChildren<MeshRenderer>();
    }

    // Start is called before the first frame update
    void Start()
    {
        for (int idx = 0; idx < renderers.Length; idx++)
        {
            renderers[idx].material = material;
        }
    }

}
