#pragma once

#include "Point.h"

class Map
{
public:
	Map(const std::string& filename);
	~Map();

	int getWidth() const { return width; }
	int getHeight() const { return height; }

	bool isWall(int x, int y) const;

	const Point& getStartPoint() const { return startPoint; }
	int getStartX() const { return startPoint.getX(); }
	int getStartY() const { return startPoint.getY(); }

	const Point& getEndPoint() const { return endPoint; }
	int getEndX() const { return endPoint.getX(); }
	int getEndY() const { return endPoint.getY(); }

private:
	int width, height;
	bool* mapData;
	Point startPoint, endPoint;
};

