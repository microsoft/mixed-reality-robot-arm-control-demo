using UnityEngine;

public class SummonObject : MonoBehaviour
{

    public void Summon()
    {
        Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
        transform.position = inFrontOfCameraPos;
        transform.rotation = Quaternion.identity;
    }
}
