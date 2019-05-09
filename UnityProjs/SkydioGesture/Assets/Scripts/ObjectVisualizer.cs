using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectVisualizer : MonoBehaviour
{

    GameObject ObjectProbPrefab;
    public GameObject ObjectProbPrefabCube;
    public GameObject ObjectProbPrefabSphere;
    public GameObject ObjectProbPrefabCircle;

    public bool changeShape = false;
    public bool useSize = true;
    public bool useColor = true;

    public RosSharp.RosBridgeClient.PointArraySubscriber pointSubscriber;
    public RosSharp.RosBridgeClient.FloatArraySubscriber probabilitySubscriber;
    public RosSharp.RosBridgeClient.FloatArraySubscriber visualizationConfigSubscriber;

    public float footprint = 0.1f;
    public float boxScale = 1.0f;
    public float sphereScale = 1.0f;
    public float circleScale = 1.0f;


    public Gradient gradient = new Gradient();

    public AnimationCurve animationCurve;

    bool needToMakeVizualizations = true;
    bool currentlyAnimating = false;

    private GameObject[] VisualizationObjects = new GameObject[0];
    private float[] currentProbabilities;
    char[] delimiters = new char[] { ':' };

    private void Start()
    {
        Debug.Log("confirm that logging works");
        ObjectProbPrefab = ObjectProbPrefabSphere;    
    }

    void Update()
    {

        if (visualizationConfigSubscriber.SubscribedArray.Length > 0 && !currentlyAnimating)
        {
            // type
            if (visualizationConfigSubscriber.SubscribedArray[0] == 0)
            {
                ObjectProbPrefab = ObjectProbPrefabCube;
            }
            else if (visualizationConfigSubscriber.SubscribedArray[0] == 1)
            {
                ObjectProbPrefab = ObjectProbPrefabSphere;
            }
            else if (visualizationConfigSubscriber.SubscribedArray[0] == 2)
            {
                ObjectProbPrefab = ObjectProbPrefabCircle;
            }
            else
            {
                Debug.LogError("visualization config is malformed");
            }

            // use size
            if (visualizationConfigSubscriber.SubscribedArray[1] == 1)
            {
                useSize = true;
            }
            else if (visualizationConfigSubscriber.SubscribedArray[1] == 0)
            {
                useSize = false;
            }
            else
            {
                Debug.LogError("visualization config is malformed");
            }

            // use color
            if (visualizationConfigSubscriber.SubscribedArray[2] == 1)
            {
                useColor = true;
            }
            else if (visualizationConfigSubscriber.SubscribedArray[2] == 0)
            {
                useColor = false;
            }
            else
            {
                Debug.LogError("visualization config is malformed");
            }
            needToMakeVizualizations = true;
            visualizationConfigSubscriber.SubscribedArray = new float[0];
        }

        if (changeShape && !currentlyAnimating)
        {
            needToMakeVizualizations = true;
            if (ObjectProbPrefab == ObjectProbPrefabCube)
            {
                ObjectProbPrefab = ObjectProbPrefabSphere;
            }
            else if (ObjectProbPrefab == ObjectProbPrefabSphere)
            {
                ObjectProbPrefab = ObjectProbPrefabCircle;
            }
            else if (ObjectProbPrefab == ObjectProbPrefabCircle)
            {
                ObjectProbPrefab = ObjectProbPrefabCube;
            }
            else
            {
                Debug.LogError("Error: ObjectProbPrefab is not a known prefab");
            }
            changeShape = false;
        }

        if (needToMakeVizualizations)
        {
            if (pointSubscriber.SubscribedArray.Length > 0)
            {
                RemovePreviousVisualizations();
                MakeVisualizations();
                currentProbabilities = new float[pointSubscriber.SubscribedArray.Length];
                currentProbabilities.Populate(1.0f / pointSubscriber.SubscribedArray.Length);
                needToMakeVizualizations = false;
            }
        }
        else if (probabilitySubscriber.SubscribedArray.Length > 0 && !currentlyAnimating)
        {
            UpdateProbabilities();
            currentProbabilities = probabilitySubscriber.SubscribedArray;

        }
    }

    void RemovePreviousVisualizations()
    {
        foreach (GameObject visualization in VisualizationObjects)
        {
            Destroy(visualization);
        }
    }

    void MakeVisualizations()
    {
        VisualizationObjects = new GameObject[pointSubscriber.SubscribedArray.Length];
        for (int i = 0; i < pointSubscriber.SubscribedArray.Length; i++)
        {
            GameObject clone;
            clone = Instantiate(ObjectProbPrefab, transform);
            clone.transform.localPosition = pointSubscriber.SubscribedArray[i];
            VisualizationObjects[i] = clone;
        }
    }

    IEnumerator Animate(Transform vizObject, float start, float end, float duration)
    {
        currentlyAnimating = true;
        float journey = 0f;
        while (journey <= duration)
        {
            journey += Time.deltaTime;
            float percent = Mathf.Clamp01(journey / duration);
            float curvePercent = animationCurve.Evaluate(percent);
            float scaleValue = Mathf.LerpUnclamped(start, end, curvePercent);

            // change size
            if (useSize)
            {
                if (ObjectProbPrefab == ObjectProbPrefabCube)
                {
                    vizObject.localScale = new Vector3(footprint, scaleValue * boxScale, footprint);
                    vizObject.localPosition = new Vector3(vizObject.localPosition.x, vizObject.localScale.y / 2f, vizObject.localPosition.z);
                }
                else if (ObjectProbPrefab == ObjectProbPrefabSphere)
                {
                    vizObject.localScale = new Vector3(scaleValue * sphereScale, scaleValue * sphereScale, scaleValue * sphereScale);
                }
                else if (ObjectProbPrefab == ObjectProbPrefabCircle)
                {
                    vizObject.GetComponent<LineRenderer>().widthMultiplier = scaleValue * circleScale;
                }
                else
                {
                    Debug.LogError("Prefab is not expected type");
                }

            }
            else
            {
                if (ObjectProbPrefab != ObjectProbPrefabCircle)
                {
                    vizObject.localScale = new Vector3(footprint, footprint, footprint);
                }
            }

            //change color
            if (useColor)
            {
                if (ObjectProbPrefab == ObjectProbPrefabCircle)
                {
                    vizObject.GetComponent<LineRenderer>().material.color = gradient.Evaluate(scaleValue);
                }
                else
                {
                    vizObject.GetComponent<MeshRenderer>().material.color = gradient.Evaluate(scaleValue);
                }
            }
            else
            {

                if (ObjectProbPrefab == ObjectProbPrefabCircle)
                {
                    vizObject.GetComponent<LineRenderer>().material.color = Color.white;
                }
                else
                {
                    vizObject.GetComponent<MeshRenderer>().material.color = Color.white;
                }
            }
            yield return null;
        }
        currentlyAnimating = false;
    }

    void UpdateProbabilities()
    {
        for (int i = 0; i < probabilitySubscriber.SubscribedArray.Length; i++)
        {
            // get the vizualization
            Transform vizObject = VisualizationObjects[i].transform.GetChild(0);
            // start the coroutine
            StartCoroutine(Animate(vizObject, currentProbabilities[i], probabilitySubscriber.SubscribedArray[i], 0.5f));

        }
    }
}

public static class ArrayHelper
{
    public static void Populate<T>(this T[] arr, T value)
    {
        for (int i = 0; i < arr.Length; i++)
        {
            arr[i] = value;
        }
    }
}
