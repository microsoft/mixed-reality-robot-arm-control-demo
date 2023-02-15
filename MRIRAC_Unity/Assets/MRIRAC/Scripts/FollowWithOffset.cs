using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowWithOffset : MonoBehaviour
{
    [SerializeField]
    private float offsetInMeters;
    [SerializeField]
    private string direction;
    [SerializeField]
    private GameObject parent;

    // Update is called once per frame
    void Update()
    {

        gameObject.transform.rotation = parent.transform.rotation;


        switch (direction)
        {
            case "up":
                gameObject.transform.position = parent.transform.position + offsetInMeters * parent.transform.up;
                break;
            case "forward":
                gameObject.transform.position = parent.transform.position + offsetInMeters * parent.transform.forward;
                break;
            case "right":
                gameObject.transform.position = parent.transform.position + offsetInMeters * parent.transform.right;
                break;
            default:
                gameObject.transform.position = parent.transform.position + offsetInMeters * parent.transform.up;
                break;
        }

    }
}
