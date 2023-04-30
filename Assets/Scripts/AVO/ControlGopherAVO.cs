using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class ControlGopherAVO : MonoBehaviour
{
	public GameObject robot;
	public AVONavigation navigation;
	public StateReader state;
	public SurroundingDetection detector;

	// public Transform goal;
	public Vector3 goal;

	private Vector3 linearVelocity;

	public DataRecorderTest recorder;




	void Start()
	{
		// navigation.SetGoal(goal.position);
		navigation.SetGoal(goal);
		navigation.EnableAutonomy(false);
		recorder.StartRecording("testingDataCollectio2", robot);
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
