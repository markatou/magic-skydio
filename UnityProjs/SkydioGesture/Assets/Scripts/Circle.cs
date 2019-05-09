using UnityEngine;
using System.Collections;

[ExecuteInEditMode]
public class Circle : MonoBehaviour
{
    public int segments;
    public float xradius;
    public float yradius;
    LineRenderer line;

    void Start()
    {
        line = gameObject.GetComponent<LineRenderer>();
        line.positionCount = segments + 2;
        line.useWorldSpace = false;
        CreatePoints();
    }


    void CreatePoints()
    {
        line.positionCount = segments + 2;
        float x;
        float y;
        float z = 0f;

        float angle = 2f;

        for (int i = 0; i < line.positionCount; i++)
        {
            x = Mathf.Sin(Mathf.Deg2Rad * angle) * xradius;
            y = Mathf.Cos(Mathf.Deg2Rad * angle) * yradius;

            line.SetPosition(i, new Vector3(x, y, z));

            angle += (360f / segments);
        }
    }
}