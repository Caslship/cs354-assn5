#include <cmath>
#include <algorithm>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"

using namespace std;

KdTree::KdTree(BoundingBox bounds, std::vector<Geometry *> objects, int depth)
{
	_bounds = bounds;
	_objects = objects;
	_axis = depth % 3;


	// We only want as many as 5 branches or 2^5=32 leaf nodes
	if (depth >= 5)
		return;

	// If we have child nodes, sort the objects by axis for later use
	switch(_axis)
	{
		case 0:
		{
			// Sort objects by x coordinates
			sort(_objects, sortX);
			break;
		}
		case 1:
		{
			// Sort objects by y coordinates
			sort(_objects, sortY);
			break;
		}
		case 2:
		{
			// Sort objects by z coordinates
			sort(_objects, sortZ);
			break;
		}
		default:
			break;
	}

	int num_objects = objects.size();
	int median_i = num_objects / 2;

	std::vector<Geometry *> left_objects;
	for (int i = 0; i < median_i; ++i)
	{
		left_objects.push_back(objects[i]);
	}

	std::vector<Geometry *> right_objects;
	for (int i = median_i; i < num_objects; ++i)
	{
		right_objects.push_back(objects[i]);
	}

	_left = new KdTree(_bounds, left_objects, depth + 1);
	_right = new KdTree(_bounds, right_objects, depth + 1);

}

bool sortX(Geometry * a, Geometry * b)
{

}

bool sortY(Geometry * a, Geometry * b)
{

}

bool sortZ(Geometry * a, Geometry * b)
{

}

bool KdTree:intersect(ray& r, isect& i) const
{
	return bounds.intersect(r, tmin, tmax);
}

bool Geometry::intersect(ray& r, isect& i) const {
	double tmin, tmax;
	if (hasBoundingBoxCapability() && !(bounds.intersect(r, tmin, tmax))) return false;
	// Transform the ray into the object's local coordinate space
	Vec3d pos = transform->globalToLocalCoords(r.p);
	Vec3d dir = transform->globalToLocalCoords(r.p + r.d) - pos;
	double length = dir.length();
	dir /= length;
	Vec3d Wpos = r.p;
	Vec3d Wdir = r.d;
	r.p = pos;
	r.d = dir;
	bool rtrn = false;
	if (intersectLocal(r, i))
	{
		// Transform the intersection point & normal returned back into global space.
		i.N = transform->localToGlobalCoordsNormal(i.N);
		i.t /= length;
		rtrn = true;
	}
	r.p = Wpos;
	r.d = Wdir;
	return rtrn;
}

bool Geometry::hasBoundingBoxCapability() const {
	// by default, primitives do not have to specify a bounding box.
	// If this method returns true for a primitive, then either the ComputeBoundingBox() or
    // the ComputeLocalBoundingBox() method must be implemented.

	// If no bounding box capability is supported for an object, that object will
	// be checked against every single ray drawn.  This should be avoided whenever possible,
	// but this possibility exists so that new primitives will not have to have bounding
	// boxes implemented for them.
	return false;
}

Scene::~Scene() {
    giter g;
    liter l;
    tmap::iterator t;
    for( g = objects.begin(); g != objects.end(); ++g ) delete (*g);
    for( l = lights.begin(); l != lights.end(); ++l ) delete (*l);
    for( t = textureCache.begin(); t != textureCache.end(); t++ ) delete (*t).second;
}

// Get any intersection with an object.  Return information about the 
// intersection through the reference parameter.
bool Scene::intersect(ray& r, isect& i) const {
	double tmin = 0.0;
	double tmax = 0.0;
	bool have_one = false;
	typedef vector<Geometry*>::const_iterator iter;
	for(iter j = objects.begin(); j != objects.end(); ++j) {
		isect cur;
		if( (*j)->intersect(r, cur) ) {
			if(!have_one || (cur.t < i.t)) {
				i = cur;
				have_one = true;
			}
		}
	}
	if(!have_one) i.setT(1000.0);
	// if debugging,
	if (TraceUI::m_debug) intersectCache.push_back(std::make_pair(new ray(r), new isect(i)));
	return have_one;
}

TextureMap* Scene::getTexture(string name) {
	tmap::const_iterator itr = textureCache.find(name);
	if(itr == textureCache.end()) {
		textureCache[name] = new TextureMap(name);
		return textureCache[name];
	} else return (*itr).second;
}


