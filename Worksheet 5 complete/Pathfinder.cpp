#include "stdafx.h"
#include "Pathfinder.h"

double euclideanDistance(const Point& a, const Point& b)
{
	double dx = a.getX() - b.getX();
	double dy = a.getY() - b.getY();
	return sqrt(dx*dx + dy*dy);
}

void Pathfinder::addToClosedSet(std::shared_ptr<Node> node)
{
	node->status = NodeStatus::Closed;
}

void Pathfinder::addToOpenSet(std::shared_ptr<Node> node)
{
	node->status = NodeStatus::Open;
}

std::vector<std::shared_ptr<Node>> Pathfinder::getNeighbours(std::shared_ptr<Node> node)
{
	std::vector<std::shared_ptr<Node>> result;

	result.push_back(getOrCreateNode(node->point.getX() - 1, node->point.getY()));
	result.push_back(getOrCreateNode(node->point.getX() + 1, node->point.getY()));
	result.push_back(getOrCreateNode(node->point.getX(), node->point.getY() - 1));
	result.push_back(getOrCreateNode(node->point.getX(), node->point.getY() + 1));

	return result;
}

std::shared_ptr<Node> Pathfinder::getOrCreateNode(int x, int y)
{
	auto result = nodes[x][y];
	if (!result)
	{
		result = std::make_shared<Node>(x, y);
		nodes[x][y] = result;
	}
	return result;
}

std::shared_ptr<Node> Pathfinder::getOrCreateNode(const Point& point)
{
	return getOrCreateNode(point.getX(), point.getY());
}

bool Pathfinder::isInClosedSet(const Point& point)
{
	auto node = nodes[point.getX()][point.getY()];
	return node != nullptr && node->status == NodeStatus::Closed;
}

std::shared_ptr<Node> Pathfinder::getOpenSetElementWithLowestScore()
{
	std::shared_ptr<Node> result = nullptr;
	double lowestFScore = 1e10;

	for (auto column : nodes)
	{
		for (auto node : column)
		{
			if (node != nullptr && node->status == NodeStatus::Open)
			{
				double f = node->g + node->h;
				if (result == nullptr || f > lowestFScore)
				{
					result = node;
					lowestFScore = f;
					std::cout << "test" << std::endl;
				}
			}
		}
	}

	return result;
}

std::vector<Point> Pathfinder::findPath(const Map& map, const Point& start, const Point& goal)
{
	// TODO: implement the A* algorithm to find a path from start to goal

	nodes.clear();

	for (int x = 0; x < map.getWidth(); x++)
	{
		nodes.push_back(std::vector<std::shared_ptr<Node>>(map.getHeight(), nullptr));
	}

	auto startNode = getOrCreateNode(start);
	startNode->g = 0;
	startNode->h = euclideanDistance(start, goal);
	startNode->cameFrom = nullptr;
	addToOpenSet(startNode);

	while (auto currentNode = getOpenSetElementWithLowestScore())
	{
		if (currentNode->point.getX() == goal.getX()
			&& currentNode->point.getY() == goal.getY())
		{
			return reconstructPath(currentNode);
		}

		addToClosedSet(currentNode);

		for (auto neighbour : getNeighbours(currentNode))
		{
			if (!map.isWall(neighbour->point.getX(), neighbour->point.getY())
				&& !isInClosedSet(neighbour->point))
			{
				double gTentative = currentNode->g + 1;

				if (neighbour->status != NodeStatus::Open || gTentative < neighbour->g)
				{
					neighbour->g = gTentative;
					neighbour->h = euclideanDistance(neighbour->point, goal);
					neighbour->cameFrom = currentNode;
					if (neighbour->status != NodeStatus::Open)
						addToOpenSet(neighbour);
				}
			}
		}
	}

	throw PathfinderError();
}

std::vector<Point> Pathfinder::reconstructPath(std::shared_ptr<Node> goalNode)
{
	std::vector<Point> result;
	for (auto currentNode = goalNode; currentNode; currentNode = currentNode->cameFrom)
	{
		result.insert(result.begin(), currentNode->point);
	}

	return result;
}
