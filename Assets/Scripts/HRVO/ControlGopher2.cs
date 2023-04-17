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

	// public Transform goal;
	public Vector3 goal;

	private Vector3 linearVelocity;




	void Start()
	{
		// navigation.SetGoal(goal.position);
		navigation.SetGoal(goal);
		navigation.EnableAutonomy(false);
	}

	void Update()
	{
		linearVelocity = state.linearVelocity;

	}

	
}
