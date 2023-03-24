using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class ControlGopher : MonoBehaviour
{
	public GameObject robot;
	public AutoNavigation navigation;
	public StateReader state;
	public SurroundingDetection detector;

    	public bool debugVisualization = false;
	public Transform goal;

	public Vector3 linearVelocity;

	private Vector3[] humanScanPositions; // list of positions to be clustered
    	public float clusterDistanceThreshold = 0.25f; // distance threshold for clustering
    	private List<List<Vector3>> clusters = new List<List<Vector3>>(); // list of clusters

	public List<Vector3> centroids;


	void Start()
	{
		navigation.SetGoal(goal.position);
		navigation.EnableAutonomy(false);
	}

	void Update()
	{
		linearVelocity = state.linearVelocity;
		dynamicObstacles();
	}
	
	void getCentroids()
	{	
		centroids.Clear();
		foreach(List<Vector3> cluster in clusters)
		{
			Vector3 center = GetClusterCentroid(cluster);
			if (Vector3.Distance(robot.transform.position, center) > 0.5f)
		        {
				centroids.Add(center);
				// Visualization
				if (debugVisualization)
				{
				    Vector3 lineLocation = center - robot.transform.position;
				    Debug.DrawRay(robot.transform.position,lineLocation,Color.green, 1f/10);
				}
			}
		}
	}

	void dynamicObstacles()
	{
		humanScanPositions = detector.GetScanResultPositions(0, detector.samples, true);
		List<Vector3> positionsList = new List<Vector3>(humanScanPositions);
		ClusterPositions(positionsList);
		getCentroids();
	}

	private void ClusterPositions(List<Vector3> positions)
    	{
		clusters.Clear();

		// Iterate over all positions and cluster them
		foreach (Vector3 position in positions)
		{
		    bool addedToCluster = false;

		    // Iterate over all existing clusters to check if the current position should be added to any of them
		    for (int i = 0; i < clusters.Count; i++)
		    {
		        if (Vector3.Distance(position, GetClusterCentroid(clusters[i])) <= clusterDistanceThreshold)
		        {
		            clusters[i].Add(position);
		            addedToCluster = true;
		            break;
		        }
		    }

		    // If the position was not added to any existing cluster, create a new cluster for it
		    if (!addedToCluster)
		    {
		        List<Vector3> newCluster = new List<Vector3>();
		        newCluster.Add(position);
		        clusters.Add(newCluster);
		    }
		}

		// Print out the resulting clusters for debugging purposes
		
		for (int i = 0; i < clusters.Count; i++)
		{
		    Debug.Log("Cluster " + i + ": " + string.Join(", ", clusters[i]));
		}
    	}

	// Helper function to calculate the centroid of a cluster
	private Vector3 GetClusterCentroid(List<Vector3> cluster)
	{
		Vector3 centroid = Vector3.zero;
		foreach (Vector3 position in cluster)
		{
		    centroid += position;
		}
		centroid /= cluster.Count;
		return centroid;
	}
}
