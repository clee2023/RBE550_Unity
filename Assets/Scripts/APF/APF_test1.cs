using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class APF_test1 : MonoBehaviour
{
    public Transform target; // the target object (e.g. player)
    public GameObject obstacle; // the obstacle object (e.g. moving nurse)
    public AutoNavigation navigation;

    public float maxForce = 10f; // maximum force that can be applied to the agent
    public float maxVelocity = 5f; // maximum velocity that the agent can move with
    public float safeDistance = 5f; // distance at which the agent should start avoiding obstacles
    public float obstacleRadius = 1f; // radius of the obstacles in the environment

    private Rigidbody rb; // the Rigidbody component of the agent

    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
	    navigation.EnableAutonomy(false);
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 desiredVelocity = (target.position - transform.position).normalized * maxVelocity;
        Vector3 steeringForce = CalculateSteeringForce() * maxForce;
        Vector3 acceleration = steeringForce / rb.mass;

        rb.velocity += acceleration * Time.deltaTime;

        if (rb.velocity.magnitude > maxVelocity)
        {
            rb.velocity = rb.velocity.normalized * maxVelocity;
        }

        transform.LookAt(transform.position + rb.velocity);
    }

    private Vector3 CalculateSteeringForce()
    {
        Vector3 avoidanceForce = CalculateAvoidanceForce();
        Vector3 attractionForce = CalculateAttractionForce();

        return avoidanceForce + attractionForce;
    }

    private Vector3 CalculateAvoidanceForce()
    {
        Vector3 avoidanceForce = Vector3.zero;

        Collider[] obstacles = Physics.OverlapSphere(transform.position, safeDistance);
        foreach (Collider obstacle in obstacles)
        {
            if (obstacle.CompareTag("Human"))
            {
                Vector3 obstacleCenter = obstacle.bounds.center;
                float obstacleRadius = obstacle.bounds.extents.magnitude;
                float distance = Vector3.Distance(obstacleCenter, transform.position);

                if (distance < safeDistance)
                {
                    float strength = (safeDistance - distance) / safeDistance;
                    Vector3 direction = transform.position - obstacleCenter;
                    avoidanceForce += direction.normalized * strength * maxForce;
                }
            }
        }
        
        // Add avoidance force for the moving obstacle (e.g. male nurse)
        if (obstacle != null)
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
        Vector3 attractionForce = (target.position - transform.position).normalized * maxForce;

        return attractionForce;
    }
}

