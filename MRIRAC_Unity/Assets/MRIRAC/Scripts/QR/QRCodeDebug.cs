using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QRCodeDebug : MonoBehaviour
{
    // Start is called before the first frame update
    [SerializeField]
    private GameObject qrCode;
    [SerializeField]
    private GameObject pickObject;
    [SerializeField]
    private Vector3 rotation;

    // Update is called once per frame
    void Update()
    {
        pickObject.transform.position = qrCode.transform.position;
        pickObject.transform.rotation = qrCode.transform.rotation * Quaternion.Euler(rotation.x, rotation.y, rotation.z);
    }
}
