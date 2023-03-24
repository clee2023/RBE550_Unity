using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ControlGopher : MonoBehaviour
{
	public GameObject robot;
	public AutoNavigation navigation;

	public Transform goal;	

	void Start()
	{
		navigation.SetGoal(goal.position);
		navigation.EnableAutonomy(false);
	}

	void Update()
	{

	}
}
