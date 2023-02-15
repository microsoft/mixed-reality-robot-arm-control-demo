using UnityEngine;

public class DisableMeshColliders : MonoBehaviour
{
    public MeshCollider[] meshColliders;


    // Start is called before the first frame update
    void Start()
    {
        meshColliders = GetComponentsInChildren<MeshCollider>();

        for (int i = 0; i < meshColliders.Length; i++)
        {
            meshColliders[i].enabled = false;
        }

    }

}
