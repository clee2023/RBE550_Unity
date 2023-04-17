using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class HRVOAgent : MonoBehaviour
{
    // Define the desired speed and maximum acceleration of the agent
    public float desiredSpeed = 5f;
    public float maxAcceleration = 10f;
    public float maxSpeed = 100f;
    public float avoidMargin = 2f;
    public float avoidRadius = 3f;

    public List<GameObject> neighbors;

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

    // Define the velocity of the agent
    public Vector3 velocity = Vector3.zero;
    private Vector3 target = Vector3.zero;

    void Start()
    {
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


        // Initialize the velocity of the agent
        velocity = transform.forward * desiredSpeed;
    }

    void Update()
    {
        FindNeighbors();
        velocity = CalculatePreferredVelocity(target);
    }

    private void FindNeighbors()
    {
        neighbors = Scan();
    }

    public void setTarget(Vector3 goal)
    {
        target = goal;
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


    // Returns a vector that points from the position of the character to the given target
    Vector3 SteerTowards(Vector3 targetPosition)
    {
        Vector3 desiredVelocity = (targetPosition - transform.position).normalized * maxSpeed;
        return desiredVelocity - velocity;
    }

    // Calculates the avoidance force for a given obstacle
    Vector3 CalculateObstacleAvoidanceForce(GameObject neighbor)
    {
        Vector3 avoidanceForce = Vector3.zero;
        Vector3 obstaclePosition = neighbor.transform.position;
        obstaclePosition.y = transform.position.y;
        float obstacleSize = neighbor.GetComponent<CapsuleCollider>().radius;
        Vector3 obstacleToCharacter = transform.position - obstaclePosition;

        float distanceFromObstacle = obstacleToCharacter.magnitude;

        if (distanceFromObstacle < avoidRadius + obstacleSize / 2f)
        {
            Vector3 characterVelocity = velocity;
            characterVelocity.y = 0;
            Vector3 obstacleVelocity = neighbor.GetComponent<Rigidbody>().velocity;
            obstacleVelocity.y = 0;
            Vector3 relativeVelocity = characterVelocity - obstacleVelocity;

            // Calculate the tangent vector to the obstacle
            Vector3 tangentVector = new Vector3(obstacleToCharacter.z, 0, -obstacleToCharacter.x).normalized;

            // Calculate the obstacle's bounding vectors
            Vector3 boundingVector1 = obstacleToCharacter.normalized * avoidRadius;
            Vector3 boundingVector2 = tangentVector * avoidMargin;

            // Calculate the start and end points of the obstacle's bounding vectors
            Vector3 leftBound = obstaclePosition + boundingVector1 - boundingVector2;
            Vector3 rightBound = obstaclePosition + boundingVector1 + boundingVector2;

            // Check if the character is to the left or right of the obstacle
            bool isCharacterToLeft = Vector3.Dot(obstacleToCharacter, tangentVector) > 0f;

            // Calculate the avoidance force
            if (isCharacterToLeft)
            {
                // Calculate the force to steer around the left side of the obstacle
                Vector3 targetPosition = leftBound - obstacleToCharacter.normalized * avoidMargin * 2f;
                avoidanceForce = SteerTowards(targetPosition);
            }
            else
            {
                // Calculate the force to steer around the right side of the obstacle
                Vector3 targetPosition = rightBound - obstacleToCharacter.normalized * avoidMargin * 2f;
                avoidanceForce = SteerTowards(targetPosition);
            }
        }

        return avoidanceForce;
    }

    // Calculates the avoidance force for all obstacles in the scene
    Vector3 CalculateAvoidanceForce()
    {
        Vector3 avoidanceForce = Vector3.zero;

        // Loop through all obstacles in the scene and calculate the avoidance force for each one
        foreach (GameObject neighbor in neighbors)
        {
            avoidanceForce += CalculateObstacleAvoidanceForce(neighbor);
        }

        return avoidanceForce;
    }

    // Calculates the preferred velocity for the character to move towards its target
    public Vector3 CalculatePreferredVelocity(Vector3 targetPosition)
    {
        Vector3 desiredVelocity = (targetPosition - transform.position).normalized * maxSpeed;

        // Check if the desired velocity would cause the character to collide with an obstacle
        Vector3 avoidanceForce = CalculateAvoidanceForce();
        Vector3 adjustedVelocity = desiredVelocity + avoidanceForce;

        // Limit the adjusted velocity to the maximum speed
        if (adjustedVelocity.magnitude > maxSpeed)
        {
            adjustedVelocity = adjustedVelocity.normalized * maxSpeed;
        }

        adjustedVelocity.y = 0;

        return adjustedVelocity;
    }
}