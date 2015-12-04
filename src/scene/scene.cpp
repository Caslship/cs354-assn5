#include <cmath>
#include <algorithm>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"

using namespace std;

KdTree::KdTree(BoundingBox bounds, std::vector<Geometry *> objects, int depth, int max_depth)
{
	_bounds = bounds;
	_objects = NULL;
	_left = NULL;
	_right = NULL;
	_depth = depth;
	_axis = depth % 3;


	// We only want as many as 5 branches or 2^5=32 leaf nodes
	if (depth >= max_depth)
	{
		_objects = new std::vector<Geometry *>(objects); // We only care about having objects in the leaf nodes
		return;
	}

	// If we have child nodes, sort the objects by axis for later use
	switch(_axis)
	{
		case 0:
		{
			// Sort objects by x coordinates
			sort(objects, compareGeomX);
			break;
		}
		case 1:
		{
			// Sort objects by y coordinates
			sort(objects, compareGeomY);
			break;
		}
		case 2:
		{
			// Sort objects by z coordinates
			sort(objects, compareGeomZ);
			break;
		}
		default:
			break;
	}

	int num_objects = objects.size();
	int median_i = num_objects / 2;

	std::vector<Geometry *> left_objects;
	BoundingBox left_bounds(objects[0]->getBoundingBox());
	for (int i = 0; i < median_i; ++i)
	{
		left_objects.push_back(objects[i]);
		left_bounds.merge(objects[i]->getBoundingBox());
	}

	std::vector<Geometry *> right_objects;
	BoundingBox right_bounds(objects[median_i]->getBoundingBox());
	for (int i = median_i; i < num_objects; ++i)
	{
		right_objects.push_back(objects[i]);
		right_bounds.merge(objects[i]->getBoundingBox());
	}

	_left = new KdTree(left_bounds, left_objects, depth + 1, max_depth);
	_right = new KdTree(right_bounds, right_objects, depth + 1, max_depth);
}

bool KdTree::intersect(ray& r, isect& i) const
{
	return bounds.intersect(r, tmin, tmax);
}

void KdTree::insert(Geometry * object)
{
	return;
}

bool compareGeomX(const Geometry * a, const Geometry * b)
{
	return a->getBoundingBox().getMin()[0] < b->getBoundingBox().getMin()[0];
}

bool compareGeomY(const Geometry * a, const Geometry * b)
{
	return a->getBoundingBox().getMin()[1] < b->getBoundingBox().getMin()[1];
}

bool compareGeomZ(const Geometry * a, const Geometry * b)
{
	return a->getBoundingBox().getMin()[2] < b->getBoundingBox().getMin()[2];
}

void Scene::buildKdTree()
{
	kdTree = new KdTree(sceneBounds, objects, 0, MAX_DEPTH);
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


