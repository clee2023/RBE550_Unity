using System.Collections.Generic;
using UnityEngine; // Assuming Unity engine is being used


public class MLAgent : MonoBehaviour
{
    //public List<Agent> allAgents; // List of all agents in the environment
    public GameObject actor;
    public InRangeSensor sensor;

    //public Vector3 zoomies = Vector3.zero;
    public List<GameObject> neighbors = new List<GameObject>();

    void Start()
    {
        
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
        List<HRVOAgent> agentsInRange = new List<HRVOAgent>();
        neighbors.Clear();
        foreach (GameObject agent in sensor.scannedAgents)
        {
            HRVOAgent newAgent = new HRVOAgent();
            newAgent.position = agent.transform.position;
            newAgent.velocity = agent.GetComponent<Rigidbody>().velocity;;
            //Debug.Log(newAgent.velocity);

            agentsInRange.Add(newAgent);
            neighbors.Add(agent);
        }

        currentAgent.hrvo.Clear();
        foreach(HRVOAgent agent in agentsInRange)
        {
            HRVOAgent focus = agent;

            // Compute RVO for each pair of agents within range
            foreach (HRVOAgent otherAgent in agentsInRange)
            {
                if (otherAgent != currentAgent)
                {
                    Vector3 rvo = hrvoAlgorithm.ComputeRVO(currentAgent,otherAgent);
                    //Debug.Log(rvo.magnitude);

                    // // Use the RVO to update the current velocity of the current agent
                    // currentVelocity += rvo;

                    // Compute VO for visualization purposes (optional)
                    Vector3 vo = hrvoAlgorithm.ComputeVO(currentAgent, otherAgent);

                    // Debug visualization (optional)
                    // Debug.DrawRay(currentPosition, vo, Color.blue); // VO visualization
                    // Debug.DrawRay(currentPosition, rvo, Color.yellow); // RVO visualization

                    // Compute HRVO for the pair of agents
                    Vector3 hrvo = hrvoAlgorithm.ComputeHRVO(currentAgent, otherAgent);
                    //Debug.Log(hrvo.magnitude);

                    currentAgent.hrvo.Add(hrvo);
                    
                    // Use the HRVO to update the current velocity of the current agent
                    //currentVelocity += hrvo;
                    currentAgent.UpdatePreferredVelocity();

                    // Debug visualization (optional)
                    // Debug.DrawRay(currentPosition, hrvo, Color.red); // HRVO visualization
                }
            }
        }
        
    }

    public Vector3 CalculatePreferredVelocity(Vector3 goal)
    {
        currentAgent.goal = goal;
        currentAgent.UpdatePreferredVelocity();
        Vector3 velocity = currentAgent.preferredVelocity;

        Debug.DrawRay(currentAgent.position, velocity, Color.white); // HRVO visualization

        //zoomies = velocity;

        return velocity;
    }
}