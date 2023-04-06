using System.Collections.Generic;
using UnityEngine; // Assuming Unity engine is being used

public class Agent
{
    public Vector3 position; // Current position of the agent
    public Vector3 velocity; // Current velocity of the agent
    public Vector3 preferredVelocity; // Preferred velocity of the agent
    public Vector3 goal; // Goal position of the agent
    public List<Vector3> hrvo = new List<Vector3>(); // HRVO of the agent, represented as a list of velocity samples

    // // Update the velocities of each agent to their chosen preferred velocities
    // public void UpdateVelocities()
    // {
    //     UpdatePreferredVelocity();

    //     // Example logic for updating velocity based on preferred velocity
    //     // ...

    //     // Update the velocity of the agent based on the chosen preferred velocity
    //     velocity = preferredVelocity;
    // }
    
    // Update the preferred velocity based on agent's goals and constraints
    public void UpdatePreferredVelocity()
    {
        // Example logic for computing preferred velocity based on goals and constraints
        // ...
        preferredVelocity = velocity;
        // Check if the preferred velocity lies within HRVO
        if (!IsPreferredVelocityInHRVO())
        {
            // Choose a new preferred velocity that lies within HRVO
            preferredVelocity = ChooseNewPreferredVelocityFromHRVO();
        }

        if (goal != null)
        {
            Vector3 desiredVelocity = (goal - position).normalized * velocity.magnitude;

            preferredVelocity = (preferredVelocity.normalized + desiredVelocity.normalized).normalized * velocity.magnitude;
        }
    }

    // Check if the preferred velocity lies within HRVO
    private bool IsPreferredVelocityInHRVO()
    {
        foreach (Vector3 hrvoVelocity in hrvo)
        {
            if (Vector3.Dot(preferredVelocity, hrvoVelocity) >= 0f)
            {
                return true;
            }
        }

        return false;
    }

    // Choose a new preferred velocity that lies within HRVO
    private Vector3 ChooseNewPreferredVelocityFromHRVO()
    {
        Vector3 newPreferredVelocity = Vector3.zero;
        float maxDotProduct = float.MinValue;

        foreach (Vector3 hrvoVelocity in hrvo)
        {
            float dotProduct = Vector3.Dot(preferredVelocity, hrvoVelocity);

            if (dotProduct > maxDotProduct)
            {
                maxDotProduct = dotProduct;
                newPreferredVelocity = hrvoVelocity;
            }
        }

        return newPreferredVelocity;
    }

}

public class HRVOAlgorithm
{
    // public float range = 5f; // Maximum range to consider for agents
    public float agentRadius = 5f; // Radius of agents for collision avoidance
    //public float timeHorizon = 2f; // Time horizon for collision avoidance
    //public float timeStep = 0.1f; // Time step for updating velocities

    // public List<Agent> GetAgentsInRange(Vector3 currentPosition, List<Agent> allAgents)
    // {
    //     List<Agent> agentsInRange = new List<Agent>();

    //     foreach (Agent agent in allAgents)
    //     {
    //         // Check if the agent is within the specified range
    //         if (Vector3.Distance(currentPosition, agent.position) <= range)
    //         {
    //             agentsInRange.Add(agent);
    //         }
    //     }

    //     return agentsInRange;
    // }

    public Vector3 ComputeVO(Agent currentAgent, Agent otherAgent)
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

    public Vector3 ComputeRVO(Agent currentAgent, Agent otherAgent)
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

    public Vector3 ComputeHRVO(Agent currentAgent, Agent otherAgent)
    {
        // Compute the VO and RVO for the pair of agents
        Vector3 vo = ComputeVO(currentAgent, otherAgent);
        Vector3 rvo = ComputeRVO(currentAgent, otherAgent);

        // Compute the HRVO by combining the VO and RVO with the preferred velocity
        Vector3 hrvo = vo + (rvo - vo) * 0.5f + currentAgent.preferredVelocity;

        return hrvo;
    }
}

public class HRVOAgent2 : MonoBehaviour
{
    //public List<Agent> allAgents; // List of all agents in the environment
    public GameObject actor;
    private Agent currentAgent; // The current agent for which HRVO is being calculated

    private HRVOAlgorithm hrvoAlgorithm;

    public InRangeSensor sensor;

    public Vector3 zoomies = Vector3.zero;
    public List<GameObject> neighbors = new List<GameObject>();

    void Start()
    {
        currentAgent = new Agent();
        hrvoAlgorithm = new HRVOAlgorithm();
    }

    void Update()
    {
        currentAgent.position = actor.transform.position;
        currentAgent.velocity = actor.GetComponent<Rigidbody>().velocity;
        // Get the current position and velocity of the current agent
        Vector3 currentPosition = currentAgent.position;
        Vector3 currentVelocity = currentAgent.velocity;

        // Get the agents within range of the current agent
        //List<Agent> agentsInRange = hrvoAlgorithm.GetAgentsInRange(currentPosition, allAgents);
        List<Agent> agentsInRange = new List<Agent>();
        neighbors.Clear();
        foreach (GameObject agent in sensor.scannedAgents)
        {
            Agent newAgent = new Agent();
            newAgent.position = agent.transform.position;
            newAgent.velocity = agent.GetComponent<Rigidbody>().velocity;
            agentsInRange.Add(newAgent);

            neighbors.Add(agent);
        }

        // foreach(Agent agent in agentsInRange)
        // {
        //     Agent focus = agent;

        //     // Compute RVO for each pair of agents within range
        //     foreach (Agent otherAgent in agentsInRange)
        //     {
        //         if (focus == currentAgent)
        //         {
        //             if (otherAgent != currentAgent)
        //             {
        //                 // Vector3 rvo = hrvoAlgorithm.ComputeRVO(currentAgent,otherAgent);

        //                 //     // Use the RVO to update the current velocity of the current agent
        //                 // currentVelocity += rvo;

        //                 // // Compute VO for visualization purposes (optional)
        //                 // Vector3 vo = hrvoAlgorithm.ComputeVO(currentAgent, otherAgent);

        //                 // // Debug visualization (optional)
        //                 // Debug.DrawRay(currentPosition, vo, Color.yellow); // VO visualization
        //                 // Debug.DrawRay(currentPosition, rvo, Color.red); // RVO visualization

        //                 // Compute HRVO for the pair of agents
        //                 Vector3 hrvo = hrvoAlgorithm.ComputeHRVO(currentAgent, otherAgent);
        //                 Debug.Log(hrvo);

        //                 currentAgent.hrvo.Add(hrvo);
                        
        //                 // Use the HRVO to update the current velocity of the current agent
        //                 //currentVelocity += hrvo;
        //                 currentAgent.UpdatePreferredVelocity();

        //                 // Debug visualization (optional)
        //                 Debug.DrawRay(currentPosition, hrvo, Color.yellow); // HRVO visualization
        //             }
        //         }
        //         else {
        //             if (otherAgent != focus)
        //             {
        //                 // Vector3 rvo = hrvoAlgorithm.ComputeRVO(currentAgent,otherAgent);

        //                 //     // Use the RVO to update the current velocity of the current agent
        //                 // currentVelocity += rvo;

        //                 // // Compute VO for visualization purposes (optional)
        //                 // Vector3 vo = hrvoAlgorithm.ComputeVO(currentAgent, otherAgent);

        //                 // // Debug visualization (optional)
        //                 // Debug.DrawRay(currentPosition, vo, Color.yellow); // VO visualization
        //                 // Debug.DrawRay(currentPosition, rvo, Color.red); // RVO visualization

        //                     // Compute HRVO for the pair of agents
        //                 Vector3 hrvo = hrvoAlgorithm.ComputeHRVO(focus, otherAgent);

        //                 focus.hrvo.Add(hrvo);
                        
        //                 // Use the HRVO to update the current velocity of the current agent
        //                 //focus.velocity += hrvo;
        //                 focus.UpdatePreferredVelocity();

        //                 // Debug visualization (optional)
        //                 Debug.DrawRay(focus.position, hrvo, Color.blue); // HRVO visualization
        //             }
        //         }
        //     }

        // }

        foreach(Agent agent in agentsInRange)
        {
            Agent focus = agent;

            // Compute RVO for each pair of agents within range
            foreach (Agent otherAgent in agentsInRange)
            {
                if (otherAgent != currentAgent)
                {
                    // Vector3 rvo = hrvoAlgorithm.ComputeRVO(currentAgent,otherAgent);

                    //     // Use the RVO to update the current velocity of the current agent
                    // currentVelocity += rvo;

                    // // Compute VO for visualization purposes (optional)
                    // Vector3 vo = hrvoAlgorithm.ComputeVO(currentAgent, otherAgent);

                    // // Debug visualization (optional)
                    // Debug.DrawRay(currentPosition, vo, Color.yellow); // VO visualization
                    // Debug.DrawRay(currentPosition, rvo, Color.red); // RVO visualization

                    // Compute HRVO for the pair of agents
                    Vector3 hrvo = hrvoAlgorithm.ComputeHRVO(currentAgent, otherAgent);
                    Debug.Log(hrvo);

                    currentAgent.hrvo.Add(hrvo);
                    
                    // Use the HRVO to update the current velocity of the current agent
                    //currentVelocity += hrvo;
                    currentAgent.UpdatePreferredVelocity();

                    // Debug visualization (optional)
                    Debug.DrawRay(currentPosition, hrvo, Color.yellow); // HRVO visualization
                }
            }
        }
        

        // // Update the current agent's velocity
        // currentAgent.velocity = currentVelocity;
        // // // Move the current agent based on the updated velocity
        // currentAgent.position += currentVelocity * Time.deltaTime;
    }

    public Vector3 CalculatePreferredVelocity(Vector3 goal)
    {
        currentAgent.goal = goal;
        currentAgent.UpdatePreferredVelocity();
        Vector3 velocity = currentAgent.preferredVelocity;

        zoomies = velocity;

        return velocity;
    }
}