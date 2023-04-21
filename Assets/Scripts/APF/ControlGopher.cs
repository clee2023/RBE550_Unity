using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ControlGopher : MonoBehaviour
{
    public GameObject robot;
    public AutoNavigation navigation;
    public Transform goal;
    // Start is called before the first frame update
    void Start()
    {
        navigation.SetGoal(goal.position);
	navigation.EnableAutonomy(false);
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
