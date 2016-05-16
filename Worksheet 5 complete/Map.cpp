#include "stdafx.h"
#include "Map.h"

Map::Map(const std::string& filename)
{
	std::ifstream mapFile(filename);

	std::vector<std::string> lines;
	width = 0;

	while (!mapFile.eof())
	{
		std::string line;
		std::getline(mapFile, line);
		lines.push_back(line);

		if (line.length() > width)
			width = line.length();
	}

	height = lines.size();

	mapData = new bool[width * height];
	for (int y = 0; y < height; y++)
	{
		const std::string& line = lines[y];
		for (int x = 0; x < width; x++)
		{
			char character;
			if (x < line.length())
				character = line[x];
			else
				character = ' ';

			switch (character)
			{
			case '#':
				mapData[y * width + x] = true;
				break;
			case 'S':
				mapData[y * width + x] = false;
				startPoint = Point(x, y);
				break;
			case 'E':
				mapData[y * width + x] = false;
				endPoint = Point(x, y);
				break;
			default:
				mapData[y * width + x] = false;
				break;
			}
		}
	}
}

Map::~Map()
{
	delete[] mapData;
}

bool Map::isWall(int x, int y) const
{
	if (x >= 0 && x < width && y >= 0 && y < height)
		return mapData[y * width + x];
	else
		return true;
}