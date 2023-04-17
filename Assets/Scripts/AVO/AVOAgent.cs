using System.Collections.Generic;
using UnityEngine; // Assuming Unity engine is being used

public class AVOAgent
{
    public Vector3 position; // Current position of the agent
    public Vector3 velocity; // Current velocity of the agent
    public Vector3 preferredVelocity; // Preferred velocity of the agent
    public Vector3 goal; // Goal position of the agent
    public List<Vector3> AVO = new List<Vector3>(); // avo of the agent, represented as a list of velocity samples

    
    // Update the preferred velocity based on agent's goals and constraints
    public void UpdatePreferredVelocity()
    {
        // Example logic for computing preferred velocity based on goals and constraints
        // ...
        preferredVelocity = velocity;
        // Check if the preferred velocity lies within avo
        if (!IsPreferredVelocityInAVO())
        {
            // Choose a new preferred velocity that lies within AVO
            preferredVelocity = ChooseNewPreferredVelocityFromAVO();
        }

        if (goal != null)
        {
            Vector3 desiredVelocity = (goal - position).normalized * velocity.magnitude;

            preferredVelocity = (preferredVelocity.normalized + desiredVelocity.normalized).normalized * velocity.magnitude;
        }
    }

    // Check if the preferred velocity lies within AVO
    private bool IsPreferredVelocityInAVO()
    {
        foreach (Vector3 AVOVelocity in AVO)
        {
            if (Vector3.Dot(preferredVelocity, AVOVelocity) >= 0f)
            {
                return true;
            }
        }

        return false;
    }

    // Choose a new preferred velocity that lies within AVO
    private Vector3 ChooseNewPreferredVelocityFromAVO()
    {
        Vector3 newPreferredVelocity = Vector3.zero;
        float maxDotProduct = float.MinValue;

        foreach (Vector3 AVOVelocity in AVO)
        {
            float dotProduct = Vector3.Dot(preferredVelocity, AVOVelocity);

            if (dotProduct > maxDotProduct)
            {
                maxDotProduct = dotProduct;
                newPreferredVelocity = AVOVelocity;
            }
        }

        return newPreferredVelocity;
    }

}

public class AVOAlgorithm
{

    public float agentRadius = 0.38f; // Radius of agents for collision avoidance


    public Vector3 ComputeVO(AVOAgent currentAgent, AVOAgent otherAgent)
    {
        // Compute relative position and velocity between the two agents
        Vector3 relativePosition = otherAgent.position - currentAgent.position;
        Vector3 relativeVelocity = otherAgent.velocity - currentAgent.velocity;

        // Compute the perpendicular vector to the relative velocity
        Vector3 perpendicular = new Vector3(-relativeVelocity.z, 0, relativeVelocity.x).normalized;

        // Compute the VO by combining the relative position and perpendicular vector
        Vector3 vo = relativePosition + (relativeVelocity.normalized * Time.deltaTime) + (perpendicular * 0.5f);

        return vo;
    }

    public Vector3 ComputeRVO(AVOAgent currentAgent, AVOAgent otherAgent)
    {
        // Compute relative position and velocity between the two agents
        Vector3 relativePosition = otherAgent.position - currentAgent.position;
        Vector3 relativeVelocity = otherAgent.velocity - currentAgent.velocity;

        // Compute the squared distance between the two agents
        float squaredDistance = relativePosition.sqrMagnitude;

        // Compute the sum of their radii
        float sumOfRadii = 2 * agentRadius;

        // Compute the reciprocal velocity obstacle (RVO)
        Vector3 rvo = Vector3.zero;
        if (squaredDistance < sumOfRadii * sumOfRadii)
        {
            // Agents are too close, compute the RVO
            float distance = Mathf.Sqrt(squaredDistance);
            Vector3 unitVector = relativePosition.normalized;
            rvo = (relativeVelocity / Time.deltaTime) - (unitVector * (sumOfRadii - distance) / Time.deltaTime);
        }

        return rvo;
    }

    public Vector3 ComputeAVO(AVOAgent currentAgent, AVOAgent otherAgent)
    {
        // Compute the VO and RVO for the pair of agents
        Vector3 vo = ComputeVO(currentAgent, otherAgent);
        Vector3 rvo = ComputeRVO(currentAgent, otherAgent);

        // Compute the AVO by combining the VO and RVO with the preferred velocity
        Vector3 AVO = vo + (rvo - vo) * 0.5f + currentAgent.preferredVelocity;

        return AVO;
    }
}

public class AVOUnityAgent : MonoBehaviour
{
    //public List<Agent> allAgents; // List of all agents in the environment
    public GameObject actor;
    private AVOAgent currentAgent; // The current agent for which AVO is being calculated

    private AVOAlgorithm AVOAlgorithm;

    public InRangeSensor sensor;

    //public Vector3 zoomies = Vector3.zero;
    public List<GameObject> neighbors = new List<GameObject>();

    void Start()
    {
        currentAgent = new AVOAgent();
        AVOAlgorithm = new AVOAlgorithm();
    }

    void Update()
    {
        currentAgent.position = actor.transform.position;
        currentAgent.velocity = actor.GetComponent<Rigidbody>().velocity;
        // Get the current position and velocity of the current agent
        Vector3 currentPosition = currentAgent.position;
        Vector3 currentVelocity = currentAgent.velocity;

        // Get the agents within range of the current agent
        //List<Agent> agentsInRange = AVOAlgorithm.GetAgentsInRange(currentPosition, allAgents);
        List<AVOAgent> agentsInRange = new List<AVOAgent>();
        neighbors.Clear();
        foreach (GameObject agent in sensor.scannedAgents)
        {
            AVOAgent newAgent = new AVOAgent();
            newAgent.position = agent.transform.position;
            newAgent.velocity = agent.GetComponent<Rigidbody>().velocity;
            agentsInRange.Add(newAgent);

            neighbors.Add(agent);
        }

        foreach(AVOAgent agent in agentsInRange)
        {
            AVOAgent focus = agent;

            // Compute AVO for each pair of agents within range
            foreach (AVOAgent otherAgent in agentsInRange)
            {
                if (otherAgent != currentAgent)
                {

                    // Compute AVO for the pair of agents
                    Vector3 AVO = AVOAlgorithm.ComputeAVO(currentAgent, otherAgent);
                    //Debug.Log(AVO);

                    currentAgent.AVO.Add(AVO);
                    
                    // Use the AVO to update the current velocity of the current agent
                    //currentVelocity += AVO;
                    currentAgent.UpdatePreferredVelocity();

                    // Debug visualization (optional)
                    Debug.DrawRay(currentPosition, AVO, Color.yellow); // AVO visualization
                }
            }
        }
        
    }

    public Vector3 CalculatePreferredVelocity(Vector3 goal)
    {
        currentAgent.goal = goal;
        currentAgent.UpdatePreferredVelocity();
        Vector3 velocity = currentAgent.preferredVelocity;

        return velocity;
    }
}