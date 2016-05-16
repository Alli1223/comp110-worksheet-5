// Pathfinding.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "PathfindingApp.h"
#include "Map.h"
#include "Pathfinder.h"

// Change this to test your pathfinder on different maps
const char* MAP_NAME = "Maps\\spiral.txt";

PathfindingApp::PathfindingApp()
{
	// Initialise SDL
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		freopen("CON", "w", stdout);
		freopen("CON", "w", stderr);
		showSdlError("SDL_Init failed");
		return;
	}
	freopen("CON", "w", stdout);
	freopen("CON", "w", stderr);

	// Load map
	map = new Map(MAP_NAME);

	// Calculate tile size, based on window height of 800 pixels
	tileSize = 800 / map->getHeight();

	// Create a window and renderer
	window = SDL_CreateWindow("Pathfinding",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		map->getWidth() * tileSize, map->getHeight() * tileSize,
		SDL_WINDOW_SHOWN);
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);

	// Load sprites
	spriteWall = IMG_LoadTexture(renderer, "Sprites\\brickGrey.png");
	spriteMouse = IMG_LoadTexture(renderer, "Sprites\\mouse.png");
	spriteExit = IMG_LoadTexture(renderer, "Sprites\\signExit.png");

	// Check for errors in loading sprites
	if (!spriteWall || !spriteMouse || !spriteExit)
	{
		showSdlError("IMG_LoadTexture failed");
		return;
	}

	// Find the path
	Pathfinder pathfinder;
	path = pathfinder.findPath(*map, map->getStartPoint(), map->getEndPoint());
}

void PathfindingApp::run()
{
	bool running = true;

	while (running)
	{
		// Handle events
		SDL_Event ev;
		if (SDL_PollEvent(&ev))
		{
			switch (ev.type)
			{
			case SDL_QUIT:
				running = false;
				break;

			default:
				break;
			}
		}

		// Clear the screen
		SDL_SetRenderDrawColor(renderer, 100, 90, 80, 255);
		SDL_RenderClear(renderer);

		// Draw the map
		drawMap();

		// Draw the found path
		SDL_SetRenderDrawColor(renderer, 255, 255, 128, 255);
		drawPath();

		// Present the rendered display
		SDL_RenderPresent(renderer);
	}
}

void PathfindingApp::drawTile(int x, int y, SDL_Texture* texture)
{
	SDL_Rect dest;
	dest.x = x * tileSize;
	dest.y = y * tileSize;
	dest.w = tileSize;
	dest.h = tileSize;
	SDL_RenderCopy(renderer, texture, nullptr, &dest);
}

void PathfindingApp::drawMap()
{
	for (int y = 0; y < map->getHeight(); y++)
	{
		for (int x = 0; x < map->getWidth(); x++)
		{
			if (map->isWall(x, y))
			{
				drawTile(x, y, spriteWall);
			}

			if (map->getStartX() == x && map->getStartY() == y)
			{
				drawTile(x, y, spriteMouse);
			}

			if (map->getEndX() == x && map->getEndY() == y)
			{
				drawTile(x, y, spriteExit);
			}
		}
	}
}

void PathfindingApp::drawPath()
{
	// Start at the start point
	// tileSize / 2 is added to all coordinates to put them in the centre of the tile
	int lastX = map->getStartX() * tileSize + tileSize / 2;
	int lastY = map->getStartY() * tileSize + tileSize / 2;

	// Step through the path
	for (const Point& point : path)
	{
		int nextX = point.getX() * tileSize + tileSize / 2;
		int nextY = point.getY() * tileSize + tileSize / 2;

		SDL_RenderDrawLine(renderer, lastX, lastY, nextX, nextY);
		lastX = nextX;
		lastY = nextY;
		;
	}
}

PathfindingApp::~PathfindingApp()
{
	// Clean everything up

	if (spriteWall)
		SDL_DestroyTexture(spriteWall);

	if (spriteMouse)
		SDL_DestroyTexture(spriteMouse);

	if (spriteExit)
		SDL_DestroyTexture(spriteExit);

	if (renderer)
		SDL_DestroyRenderer(renderer);

	if (window)
		SDL_DestroyWindow(window);

	if (map)
		delete map;

	SDL_Quit();
}

void PathfindingApp::showSdlError(const char* title)
{
	MessageBoxA(NULL, SDL_GetError(), title, MB_OK | MB_ICONERROR);
}

int main(int argc, char* args[])
{
	PathfindingApp app;
	app.run();

	return 0;
}
