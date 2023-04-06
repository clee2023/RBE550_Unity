using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class HRVO : MonoBehaviour
{
    public float radius = 0.38f;
    public float maxSpeed = 100f;

    public List<GameObject> neighbors;
    public Vector3 newVelocity;

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
        neighbors = new List<GameObject>();

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
        FindNeighbors();
        
    }

    private void FindNeighbors()
    {
        neighbors = Scan();
    }
    
    // modified from Surrounding Detection
    private List<GameObject> Scan()
    {
        List<GameObject> scanHumans = new List<GameObject>();
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
                if (raycastHits[i].collider.gameObject.tag == "Human")
                {
                    GameObject human = raycastHits[i].collider.gameObject;
                    if (!scanHumans.Contains(human))
                    {
                        scanHumans.Add(human);
                    }
                }
                
            }
        }
        return scanHumans;
    }

    private void ComputeNewVelocity(Vector3 waypoint)
    {
        Vector3 waypointVector = waypoint - transform.position;
        Vector3 prefVelocity = waypointVector.normalized * maxSpeed;

        newVelocity = prefVelocity;

        foreach (GameObject neighbor in neighbors)
        {
            Vector3 relativePosition = neighbor.transform.position - transform.position;
            Vector3 relativeVelocity = newVelocity - neighbor.GetComponent<Rigidbody>().velocity;
            float distSq = relativePosition.sqrMagnitude;

            float combinedRadius = radius + neighbor.GetComponent<CapsuleCollider>().radius;
            float combinedRadiusSq = combinedRadius * combinedRadius;

            if (distSq < combinedRadiusSq)
            {
                // Collision detected
                Vector3 w = relativeVelocity - relativePosition / Time.deltaTime;
                Vector3 u = prefVelocity - w;
                newVelocity = w + u.normalized * maxSpeed;
            }
        }
    }

}