using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class HRVOExample : MonoBehaviour
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



    public bool active;

    // Robot
    public GameObject robot;
    public ArticulationWheelController wheelController;
    public ArmControlManager leftArmControlManager;
    public ArmControlManager rightArmControlManager;
    // nav mesh agent
    // this is not actually used, only served as the parameter container
    // for speed, angularSpeed, stoppingDistance, areMask, etc.
    public NavMeshAgent agent;
    // nav mesh obstacles
    // during auto navigation, only the base one is enabled
    public NavMeshObstacle[] navMeshObstacles;
    private Coroutine planningCoroutine;

    // Motion planning
    public float replanTime = 5f;
    private float elapsed;
    private Vector3 goal = new Vector3(0f, -100f, 0f);
    private NavMeshPath path;
    private Vector3[] waypoints = new Vector3[0];
    private int waypointIndex = 0;
    private bool rotationNeeded = true;
    /*
    // temp - could be removed after controller is implemented
    private float prevDis = 0f;
    private float errorCheckTime;
    private float errorCheckFreq = 1.0f;
    */
    
    // Visualization
    public GameObject goalPrefab;
    private GameObject goalObject;
    public LineRenderer lineRenderer;
    public bool drawPathEnabled = true;


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

        agent.enabled = false;

    }

    void Update()
    {
        FindNeighbors();


        FixedUpdate();
        // Path visualization
        if (!drawPathEnabled || waypoints.Length == 0)
        {
            lineRenderer.positionCount = 0;
            if (goalObject != null)
                Destroy(goalObject);
            return;
        }
        // Draw current point + waypoints
        lineRenderer.positionCount = (1 + waypoints.Length - waypointIndex);
        lineRenderer.SetPosition(0, transform.position);
        for (int i = 0; i < waypoints.Length - waypointIndex; ++i)
        {
            // higher for better visualization
            lineRenderer.SetPosition(1 + i, waypoints[i + waypointIndex] + 
                                            new Vector3(0f, 0.05f, 0f) ); 
        }
        // Draw goal
        if (goalObject != null && 
            goalObject.transform.position != waypoints[waypoints.Length-1])
        {
            Destroy(goalObject);
        }
        if (goalObject == null)
        {
            goalObject = Instantiate(goalPrefab,
                                     waypoints[waypoints.Length-1], 
                                     Quaternion.identity);
            Utils.SetGameObjectLayer(goalObject, "Robot", true);
        }
        
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
                newVelocity = w + u.normalized * agent.speed;
            }
        }
    }

    private void NavigateToWaypoint(Vector3 waypoint)
    {
        ComputeNewVelocity(waypoint);  
        // P controller, Kp = 2
        float Kp = (float) 0.5;
        // Errors
        Quaternion targetRotation = Quaternion.LookRotation(new Vector3(newVelocity[0], 0.0f, newVelocity[2]).normalized);
        float angleDifference = Mathf.DeltaAngle(targetRotation.eulerAngles[1], 
                                                 transform.rotation.eulerAngles[1]);
        float angularSpeed = 0;
        // Adjust rotation angle first
        if (Mathf.Abs(angleDifference) > 1) // 1° tolorance
        {
            // set angular speed
            angularSpeed = Kp * angleDifference;
            angularSpeed = Mathf.Clamp(angularSpeed, -agent.angularSpeed, agent.angularSpeed);
        }
        float lineawrSpeed = newVelocity.magnitude;
        lineawrSpeed = Mathf.Clamp(lineawrSpeed, -agent.speed, agent.speed);
        wheelController.SetRobotVelocity(lineawrSpeed, angularSpeed);
    }



    /*OLD AUTO NAVIGATION STUFF*/

    void FixedUpdate()
    {
        // Check goals
        if (waypoints.Length == 0)
            return;

        // Check replan
        elapsed += Time.fixedDeltaTime;
        if (elapsed > replanTime)
        {
            // replan
            elapsed = 0f;
            SetGoal(this.goal);
        }

        // Autonomy disabled
        if (!active)
            return;

        // Check goal
        // select tolerance
        float tolerance = 0.1f;
        if (waypointIndex == waypoints.Length - 1)
            tolerance = agent.stoppingDistance;
        // move to current waypoint
        float currentDis = (transform.position - waypoints[waypointIndex]).magnitude;
        

        // Check distance to waypoints and update motion
        if (currentDis > tolerance)
        {
            NavigateToWaypoint(waypoints[waypointIndex]);
        }
        // current waypoint reached
        else
        {
            waypointIndex ++;
            // prevDis = 0; // temp
            rotationNeeded = true;
            // Fianl goal is reached
            if (waypointIndex == waypoints.Length)
            {
                wheelController.SetRobotVelocity(0f, 0f);
                DisableAutonomy();
            }
        }
    }  

    public void EnableAutonomy(bool changeArmPose = true)
    {
        // Must have valid goal and plan first
        if (goal[1] == -100f)
        {
            Debug.Log("No valid goal is set.");
            return;
        }

        // Change arm pose
        if (changeArmPose)
        {
            bool success = ChangeArmPose(5);
            Debug.Log("Changing arm pose failed.");
            if (!success)
                return;
        }
        active = true;
    }
    private bool ChangeArmPose(int presetIndex)
    {
        bool leftSuccess = true;
        bool rightSuccess = true;
        if (leftArmControlManager != null && rightArmControlManager != null)
        {
            leftSuccess = leftArmControlManager.MoveToPreset(presetIndex);
            rightSuccess = rightArmControlManager.MoveToPreset(presetIndex);
        }
        return leftSuccess && rightSuccess;
    }

    public void DisableAutonomy()
    {
        // Init parameters
        goal = new Vector3(0f, -100f, 0f);
        path = new NavMeshPath();
        waypoints = new Vector3[0];
        SetTrajectory(path);
        active = false;
    }

    private void SetObstacleActive(bool active)
    {
        // In case arm is carrying objects
        navMeshObstacles = robot.GetComponentsInChildren<NavMeshObstacle>();
        foreach(NavMeshObstacle navMeshObstacle in navMeshObstacles)
            navMeshObstacle.carving = active;
    }

    
    public void SetGoal(Vector3 goal)
    {
        // Get closest point in the nav mesh
        NavMeshHit hit;
        if (NavMesh.SamplePosition(goal, out hit, 1f, agent.areaMask))
        {
            goal = hit.position;
            // prevent nav mesh obstacles blocking path
            if (planningCoroutine != null)
                StopCoroutine(planningCoroutine);
            // path planning
            planningCoroutine = StartCoroutine(PathPlanningCoroutine(0.5f, goal));
        }
        else
        {
            Debug.Log("The given goal is invalid.");
        }
    }
    private IEnumerator PathPlanningCoroutine(float time, Vector3 goal)
    {
        SetObstacleActive(false);
        yield return new WaitForSeconds(0.1f);
        bool pathFound = FindGlobalPath(goal);

        // path found or not
        if (pathFound)
            this.goal = goal;
        else
            Debug.Log("No path found to given goal.");
        yield return new WaitForSeconds(time);
        SetObstacleActive(true);
    }
    private bool FindGlobalPath(Vector3 goal)
    {
        if (goal[1] == -100f)
            return false;

        // Global Path finding - A*
        path = new NavMeshPath();
        NavMesh.CalculatePath(transform.position, 
                              goal, agent.areaMask, path);
        // Set trajectories
        SetTrajectory(path);
        waypointIndex = 0;
        rotationNeeded = true;
        return (path.corners.Length != 0);
    }
    private void SetTrajectory(NavMeshPath path)
    {
        // Convert path into waypoints
        waypoints = new Vector3[path.corners.Length];
        // Invalid Trajectory -> stop sign
        if (path.corners.Length == 0)
        {
            wheelController.SetRobotVelocity(0f, 0f);
            return;
        }
        // Valid -> Set up waypoints and goal
        for (int i = 0; i < path.corners.Length; ++i)
        {
            waypoints[i] = path.corners[i];
        }
    }
}