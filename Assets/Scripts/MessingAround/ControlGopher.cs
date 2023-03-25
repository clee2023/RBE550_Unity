using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

	private Vector3[] humanScanPositions; // list of positions to be clustered, all in world frame
    public float clusterDistanceThreshold = 0.5f; // distance threshold for clustering
    private List<List<Vector3>> clusters = new List<List<Vector3>>(); // list of clusters

	public List<Vector3> centroids; // centroid locations in world frame
	
	private List<Vector3> prevCentroids = new List<Vector3>(); // for speed calculations
	private float diffCentroidThreshold = 0.5f;
	public List<Vector3> speeds;

	private List<List<Vector3>> listSpeeds = new List<List<Vector3>>(); // for speed smoothing
	public int smoothingWindowSize = 5; // size of the moving average window
	public List<Vector3> smoothedSpeeds = new List<Vector3>(); // list to store smoothed speeds


	void Start()
	{
		navigation.SetGoal(goal.position);
		navigation.EnableAutonomy(false);
	}

	void Update()
	{
		linearVelocity = state.linearVelocity;
		getDynamicClusters();
		getDynamicSpeeds();
		getSmoothedSpeeds();
	}

	private void getSmoothedSpeeds()
	{
		smoothedSpeeds.Clear();
		for (int i = 0; i < listSpeeds.Count; i++)
		{
			List<Vector3> velocities = listSpeeds[i];

			// Calculate the average x, y, and z components of the velocities for each centroid
			float avgX = velocities.Select(v => v.x).Average();
			float avgY = velocities.Select(v => v.y).Average();
			float avgZ = velocities.Select(v => v.z).Average();

			// Create a new Vector3 with the smoothed velocities
			Vector3 smoothedVelocity = new Vector3(avgX, avgY, avgZ);

			smoothedSpeeds.Add(smoothedVelocity);
		}

		if(debugVisualization)
		{
			if (centroids.Count == smoothedSpeeds.Count){
				for (int i = 0; i < centroids.Count; i++){
					Debug.DrawRay(centroids[i],smoothedSpeeds[i],Color.yellow, 1f/10);
				}
			}
		}
	}
	
	// obtain dynamic obstacle clusters
	private void getDynamicClusters()
	{
		humanScanPositions = detector.GetScanResultPositions(0, detector.samples, true);
		List<Vector3> positionsList = new List<Vector3>(humanScanPositions);
		ClusterPositions(positionsList);
	}

	// obtain dynamic obstacle speeds
	private void getDynamicSpeeds()
	{
		List<Vector3> newCentroids = getCentroids();

		// Match new centroids with previous centroids
		List<Vector3> matchedCentroids = new List<Vector3>();
		List<Vector3> matchedSpeeds = new List<Vector3>();
		List<Vector3> newAppearCentroids = new List<Vector3>();

		// get new speeds from matched pairs
		for (int i = 0; i < newCentroids.Count; i++)
		{
			Vector3 newCentroid = newCentroids[i];
			float minDistance = Mathf.Infinity;
			Vector3 matchedCentroid = Vector3.zero;
			Vector3 matchedSpeed = Vector3.zero;

			// pick the closest centroid
			for (int j = 0; j < prevCentroids.Count; j++)
			{
				Vector3 prevCentroid = prevCentroids[j];
				float distance = Vector3.Distance(newCentroid, prevCentroid);
				if (distance < minDistance)
				{
					minDistance = distance;
					matchedCentroid = prevCentroid;
					matchedSpeed = speeds[j];
				}
			}

			// what to do depending on distance
			if (minDistance < diffCentroidThreshold)
			{
				matchedCentroids.Add(matchedCentroid);
				matchedSpeeds.Add((newCentroid - matchedCentroid) / Time.deltaTime);
			}
			else
			{
				newAppearCentroids.Add(newCentroid);
			}
		}

		// Add disappeared centroids
		List<Vector3> disappearedCentroids = new List<Vector3>();
		List<Vector3> disappearedSpeeds = new List<Vector3>();
		for (int i = 0; i < prevCentroids.Count; i++)
		{
			Vector3 prevCentroid = prevCentroids[i];
			float minDistance = Mathf.Infinity;
			// pick the closest centroid
			for (int j = 0; j < newCentroids.Count; j++)
			{
				Vector3 newCentroid = newCentroids[j];
				float distance = Vector3.Distance(newCentroid, prevCentroid);
				if (distance < minDistance)
				{
					minDistance = distance;
				}
			}

			// what to do depending on distance
			if (minDistance >= diffCentroidThreshold)
			{
				disappearedCentroids.Add(prevCentroid);
				disappearedSpeeds.Add(speeds[i]); // keep old speed
			}
		}

		//Print out the listSpeeds for debugging purposes
		// for (int i = 0; i < listSpeeds.Count; i++)
		// {
		//     Debug.Log("Cluster " + i + ": " + string.Join(", ", listSpeeds[i]));
		// }

		// Update centroids and speeds
		centroids.Clear();
		speeds.Clear();
		centroids.AddRange(matchedCentroids);
		speeds.AddRange(matchedSpeeds);
		centroids.AddRange(newAppearCentroids);
		speeds.AddRange(newAppearCentroids.Select(x => Vector3.zero));
		prevCentroids = centroids.ToList();
		centroids.AddRange(disappearedCentroids);
		speeds.AddRange(disappearedSpeeds);

		// if(debugVisualization)
		// {
		// 	for (int i = 0; i < centroids.Count; i++){
		// 		Debug.DrawRay(centroids[i],speeds[i],Color.yellow, 1f/10);
		// 	}
		// }

		// Add to list of speeds for smoothing
		if (listSpeeds.Count == speeds.Count && speeds.Count != 0){
			if (listSpeeds.Count == 0){
				for(int i = 0; i < speeds.Count; i++)
				{
					listSpeeds.Add(new List<Vector3>() { speeds[i] });
				}
			}
			else{
				for(int i = 0; i < speeds.Count; i++)
				{
					//Debug.Log("Max: " + speeds.Count + ", Max2: " + listSpeeds.Count + ", Index: " + i);
					listSpeeds[i].Add(speeds[i]);

					if (listSpeeds.Count > smoothingWindowSize)
					{
						listSpeeds[i].RemoveAt(0);
					}
				}
			}
		}
		else {
			//Debug.Log("CLEARED");
			listSpeeds.Clear();
			for(int i = 0; i < speeds.Count; i++)
			{
				listSpeeds.Add(new List<Vector3>() { speeds[i] });
			}
		}

	}

	// cluster all the positions obtained from surrounding detection script
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
		// for (int i = 0; i < clusters.Count; i++)
		// {
		//     Debug.Log("Cluster " + i + ": " + string.Join(", ", clusters[i]));
		// }
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

	// To get all the centroids from the clusters
	private List<Vector3> getCentroids()
	{	
		List<Vector3> centroidList = new List<Vector3>();
		foreach(List<Vector3> cluster in clusters)
		{
			Vector3 center = GetClusterCentroid(cluster);
			if (Vector3.Distance(robot.transform.position, center) > 0.5f)
		    {
				centroidList.Add(center);

				// debug visualization
				if (debugVisualization)
				{
				    Vector3 lineLocation = center - robot.transform.position;
				    Debug.DrawRay(robot.transform.position,lineLocation,Color.green, 1f/10);
				}
			}
		}
		return centroidList;
	}
}
