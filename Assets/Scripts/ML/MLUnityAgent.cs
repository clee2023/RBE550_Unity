using System.Collections.Generic;
using UnityEngine; // Assuming Unity engine is being used


public class MLAgent
{
    public Vector3 position; // Current position of the agent
    public Vector3 velocity; // Current velocity of the agent
    
}

public class MLUnityAgent : MonoBehaviour
{
    //public List<Agent> allAgents; // List of all agents in the environment
    public GameObject actor;
    public InRangeSensor sensor;

    private MLAgent currentAgent; // The current agent for which HRVO is being calculated

    //public Vector3 zoomies = Vector3.zero;
    public List<GameObject> neighbors = new List<GameObject>();

    void Start()
    {
        currentAgent = new MLAgent();
    }

    void Update()
    {
        currentAgent.position = actor.transform.position;
        currentAgent.velocity = actor.GetComponent<Rigidbody>().velocity;
        // Get the current position and velocity of the current agent
        Vector3 currentPosition = currentAgent.position;
        Vector3 currentVelocity = currentAgent.velocity;

        // Get the agents within range of the current agent
        List<MLAgent> agentsInRange = new List<MLAgent>();
        neighbors.Clear();
        foreach (GameObject agent in sensor.scannedAgents)
        {
            MLAgent newAgent = new MLAgent();
            newAgent.position = agent.transform.position;
            newAgent.velocity = agent.GetComponent<Rigidbody>().velocity;;
            //Debug.Log(newAgent.velocity);

            agentsInRange.Add(newAgent);
            neighbors.Add(agent);
        } 
        
    }

    public Vector3 CalculatePreferredVelocity(Vector3 goal)
    {

        Vector3 velocity = Vector3.zero;

        //Debug.DrawRay(currentAgent.position, velocity, Color.white); // HRVO visualization

        //zoomies = velocity;

        return velocity;
    }
}