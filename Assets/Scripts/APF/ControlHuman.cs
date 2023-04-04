

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ControlHuman : MonoBehaviour
{
	public GameObject human;
	public CharacterNavigation navigation;

	//public Transform goal;
	public Vector3[] targetTrajectory;
	public bool loop = true;	

	void Start()
	{
		navigation.loop = loop;
		navigation.SetTrajectory(targetTrajectory);
	}

	void Update()
	{

	}
}
