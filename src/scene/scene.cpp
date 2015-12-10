#include <cmath>
#include <algorithm>

#include "scene.h"
#include "light.h"
#include "../ui/TraceUI.h"

#include <stack>

using namespace std;

KdTree::KdTree(std::vector<Geometry *>& objects, int depth) : _bounds(Vec3d(0, 0, 0), Vec3d(0, 0, 0))
{
	_objects = NULL;
	_left = NULL;
	_right = NULL;
	_axis = 0;
	_pivot = 0;

	int num_objects = objects.size();

	// Build bounding box for node
	if (num_objects > 0)
	{
		_bounds = objects[0]->getBoundingBox();
		for (int i = 1; i < num_objects; ++i)
			_bounds.merge(objects[i]->getBoundingBox());
	}

	// Bottom out recursion after a certain amount of objects or a depth has been reached
	if (!num_objects <= 20 || depth >= 12)
	{
		_objects = new std::vector<Geometry *>(objects);
		return;
	}

	// Split kd-tree by midpoint: https://blog.frogslayer.com/kd-trees-for-faster-ray-tracing-with-triangles/
	std::vector<Geometry *> left_objects;
	std::vector<Geometry *> right_objects;
	_axis = _bounds.getLongestAxis();
	_pivot = _bounds.getCenter()[_axis];

	for (int i = 0; i < num_objects; ++i)
	{
		if (objects[i]->getBoundingBox().getCenter()[_axis] < _pivot)
			left_objects.push_back(objects[i]);
		else
			right_objects.push_back(objects[i]);
	}

	// We don't want blank paritions
	if (left_objects.empty() && !right_objects.empty()) left_objects = right_objects;
	if (right_objects.empty() && !left_objects.empty()) right_objects = left_objects;

	// Add another depth level
	_left = new KdTree(left_objects, depth + 1);
	_right = new KdTree(right_objects, depth + 1);
}

bool KdTree::intersect(ray& r, isect& i) const
{
	// Stack-based traversal: http://www.keithlantz.net/2013/04/kd-tree-construction-using-the-surface-area-heuristic-stack-based-traversal-and-the-hyperplane-separation-theorem/
	double tmin;
	double tmax;
	bool intersection_found = false;

	// Do we even intersect with the node's bounding box?
	if (!bounds->intersect(r, tmin, tmax))
		return false;

	stack<KDTStackElement> kdt_stack;
	KDTStackElement kdt_stack_elem(this, tmin, tmax);
	kdt_stack.push(kdt_stack_elem)

	// Traverse kd-tree and find intersections if we have more nodes to traverse and if we haven't already found an intersection
	while(!kdt_stack.empty() && !intersection_found)
	{
		kdt_stack_elem = kdt_stack.pop();
		KdTree * node = kdt_stack_elem.node;
		tmin = kdt_stack_elem.tmin;
		tmax = kdt_stack_elem.tmax;

		// Traverse tree until we hit a leaf
		while (!node->isLeaf())
		{
			// Don't want to make too many method calls
			double pivot = node->getPivot();
			double axis = node->getAxis();
			KdTree * left = node->getLeft();
			KdTree * right = node->getRight();

			// Compute split intersection and designate near and far nodes
			double pivot_diff_origin = pivot - r.p[axis];
			double tstar = pivot_diff_origin / r.d[axis];
			bool near_is_left = (pivot_diff_origin >= 0);
			KdTree * near = (near_is_left ? left : right);
			KdTree * far = (near_is_left ? right : left);

			// Decide whether we should traverse near, far, or both
			if (tstar >= tmax)
				node = near;
			else if (tstar <= tmin)
				node = far;
			else
			{
				// Push far on stack for later traversal
				kdt_stack_elem.setParams(far, tstar, tmax);
				kdt_stack.push(kdt_stack_elem);
				node = near;
				tmax = tstar;
			}
		}

		// At leaf, find intersections with its objects
		vector<Geometry *> node_objects = node->getObjects();
		int num_node_objects = node_objects.size();
		for (int i = 0; i < num_node_objects; ++i)
		{
			isect cur;
			if (node_objects[i]->intersect(r, cur))
			{
				if (!intersection_found || (cur.t < i.t))
				{
					i = cur;
					intersection_found = true;
				}
			}
		}

		// Not really sure why this is needed
		if (!intersection_found)
			i.t = 1000.0;

		// If we fell outside of the scene boundaries then we obviously haven't hit anything
		if (i.t > tmax)
			intersection_found = false;
	}

	return intersection_found;
}

void Scene::buildKdTree()
{
	kdTree = new KdTree(objects, 0);
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

	if (kdtree != NULL)
		kdtree->intersect(r, i);
	else
	{
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
	}

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


