using RosMessageTypes.Std;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class SetText : MonoBehaviour
{

    [SerializeField]
    private string numberOfObstaclesTopic;
    TextMeshPro textMeshPro;


    void Awake()
    {
        textMeshPro = GetComponent<TextMeshPro>();
    }

    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<StringMsg>(numberOfObstaclesTopic, UpdateText);
    }

    void UpdateText(StringMsg stringMsg)
    {
        textMeshPro.text = "There are " + stringMsg.data + " collison objects in the planning scene";
    }
}
