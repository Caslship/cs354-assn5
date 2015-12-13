/*
* Assignment #6
* Name: Jason Palacios
* UT EID: jap4839
* UTCS: jason777
*/

// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>

extern TraceUI* traceUI;

#include <iostream>
#include <fstream>

using namespace std;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

Vec3d RayTracer::trace(double x, double y)
{
  // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();
  ray r(Vec3d(0,0,0), Vec3d(0,0,0), ray::VISIBILITY);
  scene->getCamera().rayThrough(x,y,r);
  Vec3d ret = traceRay(r, traceUI->getDepth());
  ret.clamp();
  return ret;
}

Vec3d RayTracer::tracePixel(int i, int j)
{
	Vec3d col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	// Anti-aliasing
	int num_aa_samples_sqrt = traceUI->getAASampleSqrt();
	if (num_aa_samples_sqrt > 1)
	{
		// For supersampling, a pixel is partitioned into a sqrt(num_samples) by sqrt(num_samples) grid - we need to find an increment to traverse each sample in this grid
		// The increment can be defined by assuming the image now has a resolution of (width * height * num_samples) and finding the increments of a normal pixel in that resolution
		const double x_aa_sample_inc = (1.0 / ((double)buffer_width * (double)num_aa_samples_sqrt));
		const double y_aa_sample_inc = (1.0 / ((double)buffer_height * (double)num_aa_samples_sqrt));

		// Traverse pixel like a grid and add up all the colors at each sample
		for (int y_aa_sample = 0; y_aa_sample < num_aa_samples_sqrt; ++y_aa_sample)
		{
			double sample_y = y + ((double)y_aa_sample * y_aa_sample_inc);
			for (int x_aa_sample = 0; x_aa_sample < num_aa_samples_sqrt; ++x_aa_sample)
			{
				double sample_x = x + ((double)x_aa_sample * x_aa_sample_inc);
				col += trace(sample_x, sample_y);
			}

		}

		// Average all the colors
		col /= (num_aa_samples_sqrt * num_aa_samples_sqrt);
	}
	else
	{
		// No anti-aliasing
		col = trace(x, y);
	}

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
Vec3d RayTracer::traceRay(ray& r, int depth)
{
	isect i;

	if(scene->intersect(r, i)) 
	{
		// YOUR CODE HERE

		// An intersection occurred!  We've got work to do.  For now,
		// this code gets the material for the surface that was intersected,
		// and asks that material to provide a color for the ray.  

		// This is a great place to insert code for recursive ray tracing.
		// Instead of just returning the result of shade(), add some
		// more steps: add in the contributions from reflected and refracted
		// rays.

		const Material& m = i.getMaterial();
		Vec3d color = m.shade(scene, r, i);

		// If we've reached the end of recursion, return the color of the fragment that we intersected
		if (!depth)
			return color;

		Vec3d p = r.at(i.t);
		Vec3d nV = r.d;

		// Don't bother with reflection unless kr vector isn't the 0 vector
		if (!m.kr(i).iszero())
		{
			// Find the reflection of the view vector about the normal
			Vec3d R = (nV - (2.0 * i.N) * (nV * i.N));
			R.normalize();

			// Build and trace reflection ray
			ray reflect_ray(p, R, ray::REFLECTION);
			Vec3d reflect_color = traceRay(reflect_ray, depth - 1);

			// Add reflection ray's color
			color += prod(reflect_color, m.kr(i));
		}

		// Don't bother with refraction unless kt vector isn't the 0 vector
		if (!m.kt(i).iszero())
		{
			// Determine status of current ray
			Vec3d V = -1.0 * nV;
			double cos_i = (i.N * V);
			bool entering_obj = (cos_i > 0.0);
			bool exiting_obj = (cos_i < 0.0);

			// Build index of refraction
			double n = (entering_obj 
				? 1.0 / m.index(i)
				: (exiting_obj
					? m.index(i)
					: 0.0)
				);

			// We need to adjust the normal if the ray from inside the obejct
			Vec3d N = (entering_obj
				? i.N
				: (exiting_obj
					? -1.0 * i.N
					: Vec3d(0.0, 0.0, 0.0))
				);

			double cos_t_sq = (1.0 - n * n * (1 - cos_i * cos_i));

			// Only use refraction when we don't have Total Internal Reflection
			if (cos_t_sq > 0.0 && (entering_obj || exiting_obj))
			{
				// Find refraction vector
				double cos_t = sqrt(cos_t_sq);
				Vec3d T = (((n * cos_i) - cos_t) * N) - (n * V);
				T.normalize();

				// Build and trace refraction ray
				ray refract_ray(p, T, ray::REFRACTION);
				Vec3d refract_color = traceRay(refract_ray, depth - 1);

				// Add refraction ray's color
				color += prod(refract_color, m.kt(i));
			}
		}

		return color;
	} 
	else 
	{
		// No intersection.  This ray travels to infinity, so we color it according to the background color.
		if (traceUI->usingCubeMap() && traceUI->gotCubeMap())
		{
			// Cube-mapping - see wherever our ray intersects with the cube map and color our pixel using that
			return cubemap->getColor(r);
		}
		else
		{
			// No background
			return Vec3d(0.0, 0.0, 0.0);
		}
			
	}
}

RayTracer::RayTracer()
	: scene(0), buffer(0), buffer_width(256), buffer_height(256), m_bBufferReady(false), cubemap(0)
{}

RayTracer::~RayTracer()
{
	if (cubemap)
		delete cubemap;
	
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
    Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if( !sceneLoaded() ) return false;

	scene->buildKdTree();

	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

