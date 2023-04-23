using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class APFAgent : MonoBehaviour
{
    public Vector3 target = Vector3.zero; // the target object (e.g. player)
    public GameObject agent; // what the script is on
    public List<GameObject> obstacles = new List<GameObject>(); // the obstacle object (e.g. moving nurse)
    //public AutoNavigation navigation;

    public float maxForce = 10f; // maximum force that can be applied to the agent
    public float maxVelocity = 5f; // maximum velocity that the agent can move with
    public float safeDistance = 2.5f; // distance at which the agent should start avoiding obstacles
    public float obstacleRadius = 0.38f; // radius of the obstacles in the environment

    private Rigidbody rb; // the Rigidbody component of the agent
    public Vector3 preferredVelocity = Vector3.zero;

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
	    //navigation.EnableAutonomy(false);
        GameObject[] allObjects = GameObject.FindObjectsOfType<GameObject>();

        foreach (GameObject obj in allObjects)
        {
            if (obj != agent)
            {
                obstacles.Add(obj);
            }
        }

    }

    // Update is called once per frame
    void Update()
    {
        Vector3 desiredVelocity = (target - agent.transform.position).normalized * maxVelocity;
        Vector3 steeringForce = CalculateSteeringForce() * maxForce;
        Vector3 acceleration = steeringForce / rb.mass;

        preferredVelocity += acceleration * Time.deltaTime;

        float preferredVelocityMg = preferredVelocity.magnitude;
        preferredVelocity = Mathf.Clamp(preferredVelocityMg, -2f, 2f) * preferredVelocity.normalized;
        
    }

    public Vector3 CalculatePreferredVelocity(Vector3 waypoint){
        target = waypoint;
        return preferredVelocity;
    }

    private Vector3 CalculateSteeringForce()
    {
        Vector3 attractionForce = CalculateAttractionForce();
        Vector3 avoidanceForce = Vector3.zero;

        // Calculate a dynamic safe distance based on the distance between the robot and its target
        float dynamicSafeDistance = Mathf.Lerp(10f, 1f, Vector3.Distance(transform.position, target) / 50f);

        foreach (GameObject obstacle in obstacles){
            avoidanceForce += CalculateAvoidanceForce(obstacle, dynamicSafeDistance);
        }

        return avoidanceForce/2 + attractionForce*10;
    }

    private Vector3 CalculateAvoidanceForce(GameObject obstacle, float safeDistance)
    {
        Vector3 avoidanceForce = Vector3.zero;

        if (obstacle != null && obstacle.GetComponent<Collider>() != null && obstacle.GetComponent<Collider>().bounds != null)
        {
            Vector3 obstacleCenter = obstacle.transform.position;
            float obstacleRadius = obstacle.GetComponent<Collider>().bounds.extents.magnitude;
            float distance = Vector3.Distance(obstacleCenter, transform.position);

            if (distance < safeDistance)
            {
                float strength = (safeDistance - distance) / safeDistance;
                Vector3 direction = transform.position - obstacleCenter;
                avoidanceForce += direction.normalized * strength * maxForce;
            }
        }

        return avoidanceForce;
    }


    private Vector3 CalculateAttractionForce()
    {
        Vector3 attractionForce = (target - transform.position).normalized * maxForce;

        // // Add a random jitter to the attraction force
        // float jitter = 0.1f;
        // attractionForce += new Vector3(Random.Range(-jitter, jitter), 0f, Random.Range(-jitter, jitter));

        return attractionForce;
    }
}

