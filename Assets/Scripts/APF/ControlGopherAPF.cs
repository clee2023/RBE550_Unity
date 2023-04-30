using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlGopherAPF : MonoBehaviour
{
    public GameObject robot;
    public APFNavigation navigation;

    // public Transform goal;
    public Vector3 goal;

	public DataRecorderTest recorder;

    // Start is called before the first frame update
    void Start()
    {
        navigation.SetGoal(goal);
	    navigation.EnableAutonomy(false);
        recorder.StartRecording("testingDataCollection3", robot);
    }


    // Update is called once per frame
    void Update()
    {
        float tolerance = 0.1f;
		if ((robot.transform.position - goal).magnitude < tolerance)
		{
			recorder.StopRecording();
		}
    }
}
