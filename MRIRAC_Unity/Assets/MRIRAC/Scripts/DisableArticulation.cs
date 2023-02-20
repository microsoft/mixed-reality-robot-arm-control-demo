using UnityEngine;

public class DisableArticulation : MonoBehaviour
{
    ArticulationBody[] articulationBodies;

    // Start is called before the first frame update
    void Start()
    {
        articulationBodies = GetComponentsInChildren<ArticulationBody>();


        for (int i = 0; i < articulationBodies.Length; i++)
        {
            articulationBodies[i].enabled = false;
        }
    }

}
