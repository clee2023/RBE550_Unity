using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DistanceTrack : MonoBehaviour
{
    public GameObject agent; // what the script is on
    public float distanceTraveled = 0f;
    private Vector3 prevPosition;

    void Start(){
        prevPosition = agent.transform.position;
    }

    void Update(){
        Vector3 currentPosition = agent.transform.position;
        float distance = (currentPosition-prevPosition).magnitude;
        distanceTraveled += distance;
        prevPosition = currentPosition;
    }
   

}

