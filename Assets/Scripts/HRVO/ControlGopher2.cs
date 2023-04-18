using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class ControlGopher2 : MonoBehaviour
{
	public GameObject robot;
	public HRVONavigation navigation;
	public StateReader state;
	public SurroundingDetection detector;

	public DataRecorderTest recorder;

	// public Transform goal;
	public Vector3 goal;

	private Vector3 linearVelocity;


	void Start()
	{
		// navigation.SetGoal(goal.position);
		navigation.SetGoal(goal);
		navigation.EnableAutonomy(false);
		recorder.StartRecording("testingDataCollection", robot);
	}

	void Update()
	{
		linearVelocity = state.linearVelocity;

        float tolerance = 0.1f;
		if ((robot.transform.position - goal).magnitude < tolerance)
		{
			recorder.StopRecording();
		}

	}

	
}
