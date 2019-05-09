using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;

public class POMDPControl : MonoBehaviour
{

    public MLInputController controller;

    public GameObject CoordinateConverter;
    public GameObject VisibilityToggle;
    public GameObject PointerTip;
    public Meshing Meshing;
    public GameObject RosConnector;
    public GameObject RaycastHitPoint;
    public GameObject sphere1;
    public GameObject sphere2;


    private Vector3 end;
    private Vector3 origin;
    private bool bumperDown;
    private bool homeTap;
    private bool turnOff = true;
    private float u;
    private bool ready = false;

    private void Awake()
    {
        MLInput.OnControllerButtonDown += OnButtonDown;
    }

    void Start()
    {
        MLInput.Start();
        controller = MLInput.GetController(MLInput.Hand.Left);
    }

    void OnDestroy()
    {
        MLInput.Stop();
    }

    void Update()
    {
        transform.position = controller.Position;
        transform.rotation = controller.Orientation;
        Debug.Log("T" + transform.position);

        if (bumperDown)
        {
            ToggleVisualizations();
            bumperDown = false;
        }

        if (controller.TriggerValue > 0.99f)
        {
            origin = controller.Position;
            end = PointerTip.transform.position;
            sphere1.transform.position = origin;
            sphere2.transform.position = end;

            CoordinateConverter.transform.position = PointerTip.transform.position;
            Vector3 lookAtTarget = new Vector3(PointerTip.transform.parent.transform.position.x, CoordinateConverter.transform.position.y, PointerTip.transform.parent.transform.position.z);
            CoordinateConverter.transform.LookAt(lookAtTarget);
        }

        if (controller.Touch1PosAndForce.z > 0.0)
        {
            float Y = controller.Touch1PosAndForce.y;
            u = 0.000001f;
            if (0.2 < Y & Y < 0.9)
            {
                Debug.Log("Up!");
                CoordinateConverter.transform.position = (GetNewPoint(origin, end, u, CoordinateConverter.transform.position));
            }
            if (-0.2 > Y & Y > -0.9)
            {
                Debug.Log("Down!");
                CoordinateConverter.transform.position = GetNewPoint(origin, end, -u, CoordinateConverter.transform.position);
            }

        }
    }

    void ToggleVisualizations()
    {
        Meshing.ToggleMeshScanningAndVisibility();
        foreach (MeshRenderer mr in VisibilityToggle.GetComponentsInChildren<MeshRenderer>(true))
        {   
            mr.enabled = mr.enabled ? false : true;
        }
    }

    void OnButtonDown(byte controller_id, MLInputControllerButton button)
    {
        if ((button == MLInputControllerButton.Bumper))
        {
            bumperDown = true;
        }
        if (button == MLInputControllerButton.HomeTap)
        {
            homeTap = true;
        }
    }

    Vector3 GetNewPoint(Vector3 origin, Vector3 end, float u, Vector3 current)
    {

        float dist = Vector3.Distance(origin, end);
        Debug.Log("origin " + origin);
        Debug.Log("end " + end);
        if (u < 0)
        {
            return Vector3.MoveTowards(current, end, 0.01f);
        }    
            return Vector3.MoveTowards(current, origin, 0.01f);
        
    }
}