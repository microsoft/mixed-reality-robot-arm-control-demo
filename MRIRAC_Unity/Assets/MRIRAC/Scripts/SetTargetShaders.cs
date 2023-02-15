using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetTargetShaders : MonoBehaviour
{
    [SerializeField]
    private Material defaultMaterial;
    [SerializeField]
    private Material errorMaterial;

    [SerializeField]
    private float showErrorTime;

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
            renderers[idx].material = defaultMaterial;
        }
    }

    public IEnumerator ShowError()
    {
        for (int idx = 0; idx < renderers.Length; idx++)
        {
            renderers[idx].material = errorMaterial;
        }

        yield return new WaitForSeconds(showErrorTime);

        for (int idx = 0; idx < renderers.Length; idx++)
        {
            renderers[idx].material = defaultMaterial;
        }
    }
}
