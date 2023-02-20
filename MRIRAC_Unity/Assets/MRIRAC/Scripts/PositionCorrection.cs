using System.Collections.Generic;
using UnityEngine;

public class PositionCorrection : MonoBehaviour
{
    [SerializeField]
    private float moveDistance;

    [SerializeField]
    private GameObject controlPanel;


    public void Move(int direction)
    {
        Vector3 newPosition = transform.position;
        switch (direction)
        {
            case 0:
                newPosition += moveDistance * transform.forward;
                break;
            case 1:
                newPosition -= moveDistance * transform.forward;
                break;
            case 2:
                newPosition += moveDistance * transform.right;
                break;
            case 3:
                newPosition -= moveDistance * transform.right;
                break;
            case 4:
                newPosition += moveDistance * transform.up;
                break;
            case 5:
                newPosition -= moveDistance * transform.up;
                break;

            default:
                Debug.Log("invalid direction specified");
                break;
        }

        transform.position = newPosition;

    }

    public void SetControlPanel()
    {
        controlPanel.SetActive(true);

        controlPanel.transform.rotation = gameObject.transform.rotation * Quaternion.Euler(0f, -90f, 0f);
        controlPanel.transform.position = gameObject.transform.position + gameObject.transform.right * 0.1f;
    }
}
