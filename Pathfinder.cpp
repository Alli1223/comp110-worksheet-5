#include "stdafx.h"
#include "Pathfinder.h"
#include "PathfindingApp.h"
#include "Node.h"

/* Works out the distance to the goal from the current point */
double EuclideanDistance(Node currentNode, Node nextNode)
{
	double start = currentNode.getX() - currentNode.getY();
	double next = nextNode.getX() - nextNode.getY();
	double distance = sqrt(start + next);
	return distance;
}

std::vector<Point> Pathfinder::findPath(const Map& map, const Point& start, const Point& goal)
{
	// TODO: implement the A* algorithm to find a path from start to goal
	
	Node startNode(start);
	Node endNode(goal);

	std::vector<std::string> CheckNeighbour{ "up", "down", "left", "right" };
	startNode.cameFrom = "NA";

	// Closed Set
	std::vector<Node> closedSet;

	//Open Set
	std::vector<Node> OpenSet;


	//OpenSet.push_back(startNode);
	
	startNode.g = 0;
	startNode.h = EuclideanDistance(startNode, endNode);

	while (OpenSet.size() != 0)
	{
		double nodeScore;
		double lowestScore;
		double maxScore = 100;
		int i = 0;


		for (Node node : OpenSet)  //Loop through all the nodes in OpenSet
		{
			nodeScore = node.g + node.h;
			if (nodeScore < maxScore)
			{
				lowestScore = nodeScore;
			}
		}

		Node currentNode = OpenSet[i];

		//if there is no distance left to travel then reconstruct path
		if (OpenSet[i].h == 0)
		{
			return reconstructPath(endNode);
		}

		//remove current node from openset
		//add currentnode to closed set
		closedSet.push_back(OpenSet[i]);
		OpenSet.erase(OpenSet.begin() + 1);
	}

	std::vector<Point> result;
	return result;
}

std::vector<Point> Pathfinder::reconstructPath(Node endNode)
{
	std::vector<Point> path;
	Node currentNode = endNode;
}