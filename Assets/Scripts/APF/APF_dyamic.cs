using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class APF_dyamic : MonoBehaviour
{
    // The goal object to move towards
    public Transform goal;

    // The speed at which the object moves
    public float speed = 5f;

    // The minimum distance to the goal before the object stops moving
    public float minDistance = 0.5f;

    // The repulsion radius for obstacles
    public float repulsionRadius = 2f;

    // The maximum repulsion force from an obstacle
    public float maxRepulsionForce = 10f;

    // The list of obstacles in the scene
    public List<Transform> obstacles = new List<Transform>();

    // The layer mask for detecting obstacles
    public LayerMask obstacleLayerMask;

    // The list of dynamic obstacles in the scene
    public List<Transform> dynamicObstacles = new List<Transform>();

    // The layer mask for detecting dynamic obstacles
    public LayerMask dynamicObstacleLayerMask;

    // The maximum repulsion force from a dynamic obstacle
    public float maxDynamicRepulsionForce = 20f;

    // The maximum speed of a dynamic obstacle
    public float maxDynamicObstacleSpeed = 10f;

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        Vector3 repulsionForce = Vector3.zero;

        // Calculate repulsion force from obstacles
        foreach (Transform obstacle in obstacles)
        {
            Vector3 obstacleDirection = transform.position - obstacle.position;
            float distanceToObstacle = obstacleDirection.magnitude;

            if (distanceToObstacle < repulsionRadius)
            {
                float normalizedDistance = distanceToObstacle / repulsionRadius;
                float repulsionForceMagnitude = maxRepulsionForce * (1 - Mathf.Pow(normalizedDistance, 2));
                repulsionForce += obstacleDirection.normalized * repulsionForceMagnitude;
            }
        }

        // Calculate repulsion force from dynamic obstacles
        foreach (Transform dynamicObstacle in dynamicObstacles)
        {
            Vector3 dynamicObstacleDirection = transform.position - dynamicObstacle.position;
            float distanceToDynamicObstacle = dynamicObstacleDirection.magnitude;

            if (distanceToDynamicObstacle < repulsionRadius)
            {
                float normalizedDistance = distanceToDynamicObstacle / repulsionRadius;
                float repulsionForceMagnitude = maxDynamicRepulsionForce * (1 - Mathf.Pow(normalizedDistance, 2));
                repulsionForce += dynamicObstacleDirection.normalized * repulsionForceMagnitude;
            }
        }

        // Add a force towards the goal, while also adding repulsion force from obstacles
        Vector3 directionToGoal = goal.position - transform.position;
        Vector3 totalForce = directionToGoal.normalized * speed + repulsionForce;
        rb.AddForce(totalForce);

        // Limit the speed of the object
        rb.velocity = Vector3.ClampMagnitude(rb.velocity, speed);

        // Rotate the object to face the direction of movement
        if (rb.velocity != Vector3.zero)
        {
            transform.rotation = Quaternion.LookRotation(rb.velocity, transform.up);
        }

        // Stop moving if the object is close enough to the goal
        if (Vector3.Distance(transform.position, goal.position) < minDistance)
        {
            rb.velocity = Vector3.zero;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        // Add dynamic obstacles to the

