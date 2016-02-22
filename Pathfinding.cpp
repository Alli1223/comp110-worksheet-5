// Pathfinding.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "Pathfinding.h"

int main(int argc, char* args[])
{
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		std::cout << "SDL_Init failed: " << SDL_GetError() << std::endl;
	}
	else
	{
		SDL_Window* window = SDL_CreateWindow("My first SDL game", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_SHOWN);

		if (window == nullptr)
		{
			std::cout << "SDL_CreateWindow failed: " << SDL_GetError() << std::endl;
		}
		else
		{
			SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);

			if (renderer == nullptr)
			{
				std::cout << "SDL_CreateRenderer failed: " << SDL_GetError() << std::endl;
			}
			else
			{
				int x = 0;

				bool running = true;

				while (running)
				{
					SDL_Event ev;
					if (SDL_PollEvent(&ev))
					{
						switch (ev.type)
						{
						case SDL_QUIT:
							running = false;
							break;

						case SDL_KEYDOWN:
							if (ev.key.keysym.sym == SDLK_SPACE)
								x = 0;
							break;

						default:
							break;
						}
					}

					SDL_SetRenderDrawColor(renderer, 255, 128, 0, 255);
					SDL_RenderClear(renderer);

					SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
					SDL_RenderDrawLine(renderer, x, 100, 500, 300);
					SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
					SDL_RenderDrawLine(renderer, 500, 100, x, 300);

					SDL_RenderPresent(renderer);

					x = (x + 1) % 800;
				}

				SDL_DestroyRenderer(renderer);
			}
			SDL_DestroyWindow(window);
		}

		SDL_Quit();
	}

	return 0;
}
