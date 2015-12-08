#include <iostream>
#include <time.h>
#include <stdarg.h>

#include <assert.h>

#include "CommandLineUI.h"
#include "../fileio/bitmap.h"

#include "../RayTracer.h"

#include <thread>
#include <cmath>

using namespace std;

// The command line UI simply parses out all the arguments off
// the command line and stores them locally.
CommandLineUI::CommandLineUI( int argc, char* const* argv )
	: TraceUI()
{
	int i;

	progName=argv[0];

	while( (i = getopt( argc, argv, "tr:w:h:" )) != EOF )
	{
		switch( i )
		{
			case 'r':
				m_nDepth = atoi( optarg );
				break;

			case 'w':
				m_nSize = atoi( optarg );
				break;
			default:
			// Oops; unknown argument
			std::cerr << "Invalid argument: '" << i << "'." << std::endl;
			usage();
			exit(1);
		}
	}

	if( optind >= argc-1 )
	{
		std::cerr << "no input and/or output name." << std::endl;
		exit(1);
	}

	rayName = argv[optind];
	imgName = argv[optind+1];
}

void CommandLineUI::traceThreadFunc(const int start_x, const int end_x, const int start_y, const int end_y)
{
	for(int y = start_y; y < end_y; ++y)
		for(int x = start_x; x < end_x; ++x)
			raytracer->tracePixel(x, y);
}

int CommandLineUI::run()
{
	assert( raytracer != 0 );
	raytracer->loadScene( rayName );

	if( raytracer->sceneLoaded() )
	{
		int width = m_nSize;
		int height = (int)(width / raytracer->aspectRatio() + 0.5);

		raytracer->traceSetup( width, height );

		clock_t start, end;

		// Handle tracing via multiple threads if possible
		const int num_threads = thread::hardware_concurrency();

		// Can we even use multiple-threads?
		if (num_threads > 1)
		{
			// We need to break up the image into a grid of regions where there are the same number of regions for length and height
			// Each region is handled by a thread so that each thread has approximately the same amount of work (edge regions may be larger)
			const int num_threads_sqrt = sqrt(num_threads);
			const int x_thread_sample_inc = width / num_threads_sqrt;
			const int y_thread_sample_inc = height / num_threads_sqrt;

			vector<thread> trace_threads;

			start = clock();

			// Create a thread to handle ray tracing for each region
			for (int y_thread_sample = 0; y_thread_sample < num_threads_sqrt; ++y_thread_sample)
			{
				bool not_last_row = (y_thread_sample != (num_threads_sqrt - 1));
				int y_thread_sample_start = y_thread_sample * y_thread_sample_inc;
				int y_thread_sample_end = (not_last_row ? y_thread_sample_start + y_thread_sample_inc : height); // Ensure that the last row region covers all remaining pixels height-wise

				for (int x_thread_sample = 0; x_thread_sample < num_threads_sqrt; ++x_thread_sample)
				{
					bool not_last_column = (x_thread_sample != (num_threads_sqrt - 1));
					int x_thread_sample_start = x_thread_sample * x_thread_sample_inc;
					int x_thread_sample_end = (not_last_column ? x_thread_sample_start + x_thread_sample_inc : width); // Ensure that the last column region covers remaining pixels width-wise

					// Run thread given bounds for region
					trace_threads[i].push_back(thread(x_thread_sample_start, x_thread_sample_end, y_thread_sample_start, y_thread_sample_end));
				}
			}

			// Wait for all threads to finish
			for (int i = 0; i < num_threads; ++i)
				trace_threads[i].join();

			end = clock();
		}
		else
		{
			// Regular single-thread ray tracing
			start=clock();

			for (int j = 0; j < height; ++j)
				for (int i = 0; i < width; ++i)
					raytracer->tracePixel(i, j);

			end=clock();
		}

		// save image
		unsigned char* buf;

		raytracer->getBuffer(buf, width, height);

		if (buf)
			writeBMP(imgName, width, height, buf);

		double t=(double)(end-start)/CLOCKS_PER_SEC;
//		int totalRays = TraceUI::resetCount();
//		std::cout << "total time = " << t << " seconds, rays traced = " << totalRays << std::endl;
        return 0;
	}
	else
	{
		std::cerr << "Unable to load ray file '" << rayName << "'" << std::endl;
		return( 1 );
	}
}

void CommandLineUI::alert( const string& msg )
{
	std::cerr << msg << std::endl;
}

void CommandLineUI::usage()
{
	std::cerr << "usage: " << progName << " [options] [input.ray output.bmp]" << std::endl;
	std::cerr << "  -r <#>      set recursion level (default " << m_nDepth << ")" << std::endl; 
	std::cerr << "  -w <#>      set output image width (default " << m_nSize << ")" << std::endl;
}
