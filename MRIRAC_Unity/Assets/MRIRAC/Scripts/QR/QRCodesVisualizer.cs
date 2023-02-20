// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

using System.Collections.Generic;
using UnityEngine;

namespace Microsoft.MixedReality.SampleQRCodes
{
    public class QRCodesVisualizer : MonoBehaviour
    {
        public GameObject qrCodePrefab;

        private SortedDictionary<System.Guid, GameObject> qrCodesObjectsList;
        private bool clearExisting = false;

        [SerializeField]
        private GameObject robot;
        [SerializeField]
        private GameObject localizeButton;
        [SerializeField]
        private GameObject target;
        [SerializeField]
        private GameObject pickTarget;

        public bool pickActive = false;
        [SerializeField]
        private GameObject hologramObstacles;
        [SerializeField]
        private GameObject pickObjectPrefab;
        GameObject pickObject;
        public string trackedCodes;
        [SerializeField]
        private Vector3 robotRotation;
        [SerializeField]
        private Vector3 robotOffset;
        [SerializeField]
        private Vector3 buttonOffset;
        bool localizing;
        [SerializeField]
        private string pickID;
        System.Guid pickGUID;

        struct ActionData
        {
            public enum Type
            {
                Added,
                Updated,
                Removed
            };
            public Type type;
            public Microsoft.MixedReality.QR.QRCode qrCode;

            public ActionData(Type type, Microsoft.MixedReality.QR.QRCode qRCode) : this()
            {
                this.type = type;
                qrCode = qRCode;
            }
        }

        private Queue<ActionData> pendingActions = new Queue<ActionData>();

        // Use this for initialization
        void Start()
        {
            Debug.Log("QRCodesVisualizer start");
            qrCodesObjectsList = new SortedDictionary<System.Guid, GameObject>();

            localizing = true;

            QRCodesManager.Instance.QRCodesTrackingStateChanged += Instance_QRCodesTrackingStateChanged;
            QRCodesManager.Instance.QRCodeAdded += Instance_QRCodeAdded;
            QRCodesManager.Instance.QRCodeUpdated += Instance_QRCodeUpdated;
            QRCodesManager.Instance.QRCodeRemoved += Instance_QRCodeRemoved;
            if (qrCodePrefab == null)
            {
                throw new System.Exception("Prefab not assigned");
            }
        }
        private void Instance_QRCodesTrackingStateChanged(object sender, bool status)
        {
            if (!status)
            {
                clearExisting = true;
            }
        }

        private void Instance_QRCodeAdded(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
        {
            Debug.Log("QRCodesVisualizer Instance_QRCodeAdded");

            lock (pendingActions)
            {
                pendingActions.Enqueue(new ActionData(ActionData.Type.Added, e.Data));
            }
        }

        private void Instance_QRCodeUpdated(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
        {
            Debug.Log("QRCodesVisualizer Instance_QRCodeUpdated");

            lock (pendingActions)
            {
                pendingActions.Enqueue(new ActionData(ActionData.Type.Updated, e.Data));
            }
        }

        private void Instance_QRCodeRemoved(object sender, QRCodeEventArgs<Microsoft.MixedReality.QR.QRCode> e)
        {
            Debug.Log("QRCodesVisualizer Instance_QRCodeRemoved");

            lock (pendingActions)
            {
                pendingActions.Enqueue(new ActionData(ActionData.Type.Removed, e.Data));
            }
        }

        private void HandleEvents()
        {
            lock (pendingActions)
            {
                while (pendingActions.Count > 0)
                {
                    var action = pendingActions.Dequeue();
                    if (action.type == ActionData.Type.Added)
                    {
                        GameObject qrCodeObject = Instantiate(qrCodePrefab, new Vector3(0, 0, 0), Quaternion.identity);
                        qrCodeObject.GetComponent<SpatialGraphNodeTracker>().Id = action.qrCode.SpatialGraphNodeId;
                        qrCodeObject.GetComponent<QRCode>().qrCode = action.qrCode;
                        qrCodesObjectsList.Add(action.qrCode.Id, qrCodeObject); //QRcode added

                        if (qrCodesObjectsList.ContainsKey(action.qrCode.Id))
                        {
                            Destroy(qrCodesObjectsList[action.qrCode.Id]);
                            qrCodesObjectsList.Remove(action.qrCode.Id);
                        }
                        Debug.Log("Addded " + action.qrCode.Id);
                        trackedCodes += (action.qrCode.Id + "\n");
                    }

                    else if (action.type == ActionData.Type.Updated)
                    {
                        if (!qrCodesObjectsList.ContainsKey(action.qrCode.Id))
                        {
                            GameObject qrCodeObject = Instantiate(qrCodePrefab, new Vector3(0, 0, 0), Quaternion.identity);
                            qrCodeObject.GetComponent<SpatialGraphNodeTracker>().Id = action.qrCode.SpatialGraphNodeId;
                            qrCodeObject.GetComponent<QRCode>().qrCode = action.qrCode;

                            //if same key is already there, update its position
                            if (qrCodesObjectsList.ContainsKey(action.qrCode.Id))
                            {
                                Destroy(qrCodesObjectsList[action.qrCode.Id]);
                                qrCodesObjectsList.Remove(action.qrCode.Id);
                            }

                            qrCodesObjectsList.Add(action.qrCode.Id, qrCodeObject);
                        }
                    }
                    else if (action.type == ActionData.Type.Removed)
                    {
                        if (qrCodesObjectsList.ContainsKey(action.qrCode.Id))
                        {
                            Destroy(qrCodesObjectsList[action.qrCode.Id]);
                            qrCodesObjectsList.Remove(action.qrCode.Id);
                        }
                    }
                }
            }
            if (clearExisting)
            {
                clearExisting = false;
                foreach (var obj in qrCodesObjectsList)
                {
                    Destroy(obj.Value);
                }
                qrCodesObjectsList.Clear();

            }
        }

        // Update is called once per frame
        void Update()
        {
            HandleEvents();

            foreach (var qrcode in qrCodesObjectsList)
            {
                if (localizing)
                {
                    if (qrcode.Value.GetComponent<QRCode>().CodeText == "Kinova")
                    {
                        localizeButton.SetActive(true);

                        localizeButton.transform.position = qrcode.Value.gameObject.transform.position + buttonOffset;

                        robot.transform.parent = qrcode.Value.transform;
                        robot.transform.localPosition = robotOffset;
                        robot.transform.localEulerAngles = robotRotation;
                    }
                }
                if (qrcode.Value.GetComponent<QRCode>().CodeText == pickID)
                {
                    GameObject code = qrcode.Value.GetComponent<QRCode>().gameObject;
                    float size = qrcode.Value.GetComponent<QRCode>().PhysicalSize;
                    Vector3 codePosition = code.transform.position;
                    Quaternion codeOrientation = code.transform.rotation;
                    Vector3 centerPosition = codePosition + code.transform.right * size / 2 + code.transform.up * size / 2;

                    pickTarget.transform.rotation = codeOrientation;
                    Vector3 pickPosition = centerPosition + 0.16f * pickTarget.transform.up - 0.05f * pickTarget.transform.forward;
                    pickTarget.transform.position = pickPosition;

                    if (!pickActive)
                    {
                        pickObject = Instantiate(pickObjectPrefab);
                        pickObject.name = "pickObject";
                        pickObject.transform.SetParent(hologramObstacles.transform);
                        pickActive = true;
                    }
                    if (pickActive)
                    {
                        pickObject.transform.rotation = codeOrientation * Quaternion.Euler(90f, 0f, 0f);
                        pickObject.transform.position = centerPosition;
                    }
                }

            }
        }

        public void StartScan()
        {
            // start QR tracking with the press of a button
            QRCodesManager.Instance.StartQRTracking();
        }

        public void StopLocalizing()
        {
            robot.transform.parent = null;

            localizing = false;

            Vector3 inFrontOfCameraPos = (Camera.main.transform.forward * 0.5f) + Camera.main.transform.position;
            target.transform.position = inFrontOfCameraPos;

            localizeButton.SetActive(false);

        }

        public void ResetPick()
        {
            clearExisting = true;
        }
    }
}
