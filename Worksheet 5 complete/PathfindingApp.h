#pragma once

#include "resource.h"
#include "Map.h"

class PathfindingApp
{
public:
	/** Initialise the application */
	PathfindingApp();

	/** Deinitialise the application */
	~PathfindingApp();

	/** Run the main loop of the application */
	void run();

private:
	/** Size of a tile in pixels */
	int tileSize;

	SDL_Window* window;
	SDL_Renderer* renderer;

	/** Texture used to render the walls */
	SDL_Texture* spriteWall;

	/** Texture used to render the start point */
	SDL_Texture* spriteMouse;

	/** Texture used to render the end point */
	SDL_Texture* spriteExit;

	/** The map to be navigated */
	Map* map;
	
	/** The path discovered by the pathfinding algorithm */
	std::vector<Point> path;

	/** Display an error message for the last SDL API call */
	void showSdlError(const char* title);

	/** Draw a single tile. x and y are in map coordinates, *not* pixel coordinates! */
	void drawTile(int x, int y, SDL_Texture* texture);

	/** Draw the map (walls, start and end points) on screen */
	void drawMap();

	/** Draw the path discovered by the pathfinding algorithm */
	void drawPath();
};
