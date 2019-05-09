using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.MagicLeap;

public class Meshing : MonoBehaviour
{
    public Material WireframeMaterial;
    public Material BlackMaterial;
    public MLSpatialMapper _mapper;


    private void Update()
    {
        UpdateMeshMaterial();
    }

    public void ToggleMeshScanningAndVisibility()
    {
        _mapper.enabled = _mapper.enabled ? false : true;
    }

    private void UpdateMeshMaterial()
    {
        // Loop over all the child mesh nodes created by MLSpatialMapper script
        for (int i = 0; i < transform.childCount; i++)
        {
            // Get the child gameObject
            GameObject gameObject = transform.GetChild(i).gameObject;
            // Get the meshRenderer component
            MeshRenderer meshRenderer = gameObject.GetComponent<MeshRenderer>();
            // Get the assigned material
            Material material = meshRenderer.sharedMaterial;

            if (_mapper.enabled)
            {
                if (material != WireframeMaterial)
                {
                    meshRenderer.material = WireframeMaterial;
                }
            }
            else if (material != BlackMaterial)
            {
                meshRenderer.material = BlackMaterial;
            }
        }
    }
}