using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlGopherAPF : MonoBehaviour
{
    public GameObject robot;
    public APFNavigation navigation;

    // public Transform goal;
    public Vector3 goal;
    // Start is called before the first frame update
    void Start()
    {
        navigation.SetGoal(goal);
	    navigation.EnableAutonomy(false);
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
