using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class InRangeSensor : MonoBehaviour
{
    public List<GameObject> scannedAgents;

    // general
    public GameObject centerObject;
    protected Vector3 rayStartPosition;
    protected Vector3 rayStartForward;
    // scan params
    public int updateRate = 10;
    protected float scanTime;
    public int samples = 180;
    public float angleMin = -1.5708f;
    public float angleMax = 1.5708f;
    protected float angleIncrement;
    public float rangeMin = 0.1f;
    public float rangeMax = 5.0f;
    // containers
    protected RaycastHit[] raycastHits;
    protected Quaternion[] rayRotations;
    private float[] directions;


    void Start()
    {
        scannedAgents = new List<GameObject>();

        // Containers
        rayRotations = new Quaternion[samples];
        directions = new float[samples];
        // Calculate resolution based on angle limit and number of samples
        angleIncrement = (angleMax - angleMin) / (samples - 1);
        for (int i = 0; i < samples; ++i)
        {
            directions[i] = angleMin + i * angleIncrement;
            rayRotations[i] = 
                Quaternion.Euler(new Vector3(0f, directions[i] * Mathf.Rad2Deg, 0f));
        }
        // Start scanning
        scanTime = 1f / updateRate;
        //InvokeRepeating("Scan", 1f, scanTime);

    }

    void Update()
    {
        FindAgents();
        
    }

    public void FindAgents()
    {
        scannedAgents = ScanAgents();
    }
    
    // modified from Surrounding Detection
    private List<GameObject> ScanAgents()
    {
        List<GameObject> scanAgents = new List<GameObject>();
        // Cast rays towards diffent directions to find colliders
        rayStartPosition = centerObject.transform.position;
        rayStartForward = centerObject.transform.forward;
        for (int i = 0; i < samples; ++i)
        {
            // Ray angle
            Vector3 rotation = rayRotations[i] * rayStartForward;
            // Check if hit colliders within distance
            raycastHits = new RaycastHit[samples];
            if (Physics.Raycast(rayStartPosition, rotation, out raycastHits[i], rangeMax) 
                && (raycastHits[i].distance >= rangeMin)
                && (!raycastHits[i].collider.isTrigger) )
            {
                if (raycastHits[i].collider.gameObject.tag == "Human" || raycastHits[i].collider.gameObject.tag == "Robot")
                {
                    GameObject agent = raycastHits[i].collider.gameObject;
                    if (!scannedAgents.Contains(agent))
                    {
                        scannedAgents.Add(agent);
                    }
                }
                
            }
        }
        return scannedAgents;
    }

}