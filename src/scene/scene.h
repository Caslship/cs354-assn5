/*
* Assignment #6
* Name: Jason Palacios
* UT EID: jap4839
* UTCS: jason777
*/

//
// scene.h
//
// The Scene class and the geometric types that it can contain.
//

#pragma warning (disable: 4786)


#ifndef __SCENE_H__
#define __SCENE_H__

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <memory>

#include "ray.h"
#include "material.h"
#include "camera.h"
#include "bbox.h"

#include "../vecmath/vec.h"
#include "../vecmath/mat.h"

class Light;
class Scene;


class SceneElement {

public:
  virtual ~SceneElement() {}

  Scene *getScene() const { return scene; }

  // For debugging purposes, draws using OpenGL
  virtual void glDraw(int quality, bool actualMaterials, bool actualTextures) const  { }

 protected:
 SceneElement( Scene *s )
   : scene( s ) {}

  Scene *scene;
};

class TransformNode {

protected:

  // information about this node's transformation
  Mat4d    xform;
  Mat4d    inverse;
  Mat3d    normi;

  // information about parent & children
  TransformNode *parent;
  std::vector<TransformNode*> children;
    
 public:
  typedef std::vector<TransformNode*>::iterator          child_iter;
  typedef std::vector<TransformNode*>::const_iterator    child_citer;

  ~TransformNode() {
      for(child_iter c = children.begin(); c != children.end(); ++c ) delete (*c);
    }

  TransformNode *createChild(const Mat4d& xform) {
    TransformNode *child = new TransformNode(this, xform);
    children.push_back(child);
    return child;
  }
    
  // Coordinate-Space transformation
  Vec3d globalToLocalCoords(const Vec3d &v) { return inverse * v; }

  Vec3d localToGlobalCoords(const Vec3d &v) { return xform * v; }

  Vec4d localToGlobalCoords(const Vec4d &v) { return xform * v; }

  Vec3d localToGlobalCoordsNormal(const Vec3d &v) {
    Vec3d ret = normi * v;
    ret.normalize();
    return ret;
  }

  const Mat4d& transform() const		{ return xform; }

protected:
  // protected so that users can't directly construct one of these...
  // force them to use the createChild() method.  Note that they CAN
  // directly create a TransformRoot object.
 TransformNode(TransformNode *parent, const Mat4d& xform ) : children() {
      this->parent = parent;
      if (parent == NULL) this->xform = xform;
      else this->xform = parent->xform * xform;  
      inverse = this->xform.inverse();
      normi = this->xform.upper33().inverse().transpose();
    }
};

class TransformRoot : public TransformNode {
 public:
 TransformRoot() : TransformNode(NULL, Mat4d()) {}
};

// A Geometry object is anything that has extent in three dimensions.
// It may not be an actual visible scene object.  For example, hierarchical
// spatial subdivision could be expressed in terms of Geometry instances.
class Geometry : public SceneElement {

protected:
  // intersections performed in the object's local coordinate space
  // do not call directly - this should only be called by intersect()
  virtual bool intersectLocal(ray& r, isect& i ) const = 0;

public:
  // intersections performed in the global coordinate space.
  bool intersect(ray& r, isect& i) const;

  virtual bool hasBoundingBoxCapability() const;
  const BoundingBox& getBoundingBox() const { return bounds; }
  Vec3d getNormal() { return Vec3d(1.0, 0.0, 0.0); }

  virtual void ComputeBoundingBox() {
    // take the object's local bounding box, transform all 8 points on it,
    // and use those to find a new bounding box.

    BoundingBox localBounds = ComputeLocalBoundingBox();
        
    Vec3d min = localBounds.getMin();
    Vec3d max = localBounds.getMax();

    Vec4d v, newMax, newMin;

    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], min[2], 1) );
    newMax = v;
    newMin = v;
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], min[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], min[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(min[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
    v = transform->localToGlobalCoords( Vec4d(max[0], max[1], max[2], 1) );
    newMax = maximum(newMax, v);
    newMin = minimum(newMin, v);
		
    bounds.setMax(Vec3d(newMax));
    bounds.setMin(Vec3d(newMin));
  }

  // default method for ComputeLocalBoundingBox returns a bogus bounding box;
  // this should be overridden if hasBoundingBoxCapability() is true.
  virtual BoundingBox ComputeLocalBoundingBox() { return BoundingBox(); }

  virtual bool isTrimesh() const { return false; }
  virtual void buildKdTree() {}

  void setTransform(TransformNode *transform) { this->transform = transform; };
    
 Geometry(Scene *scene) : SceneElement( scene ) {}

  // For debugging purposes, draws using OpenGL
  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  // The defult does nothing; this is here because it is not required
  // that you implement this function if you create your own scene objects.
  virtual void glDrawLocal(int quality, bool actualMaterials, bool actualTextures) const { }

 protected:
  BoundingBox bounds;
  TransformNode *transform;
};

// A SceneObject is a real actual thing that we want to model in the 
// world.  It has extent (its Geometry heritage) and surface properties
// (its material binding).  The decision of how to store that material
// is left up to the subclass.
class SceneObject : public Geometry {

 public:
  virtual const Material& getMaterial() const = 0;
  virtual void setMaterial(Material *m) = 0;

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

 protected:
 SceneObject( Scene *scene )
   : Geometry( scene ) {}
};

// A simple extension of SceneObject that adds an instance of Material
// for simple material bindings.
class MaterialSceneObject : public SceneObject {

public:
  virtual ~MaterialSceneObject() { delete material; }

  virtual const Material& getMaterial() const { return *material; }
  virtual void setMaterial(Material* m)	{ delete material; material = m; }

protected:
 MaterialSceneObject(Scene *scene, Material *mat) 
   : SceneObject(scene), material(mat) {}

  Material* material;
};

template <class T>
class KdTree
{
private:
  int _axis;
  double _pivot;
  KdTree<T> * _left;
  KdTree<T> * _right;
  BoundingBox _bounds;
  std::vector<T*> * _objects;

public:
  KdTree(std::vector<T*>& objects, int depth = 0) : _bounds(Vec3d(0, 0, 0), Vec3d(0, 0, 0))
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
    if (num_objects <= 20 || depth >= 12)
    {
      _objects = new std::vector<T*>(objects);
      return;
    }

    // Split kd-tree by midpoint of longest axis: https://blog.frogslayer.com/kd-trees-for-faster-ray-tracing-with-triangles/
    std::vector<T*> left_objects;
    std::vector<T*> right_objects;
    _axis = _bounds.getLongestAxis();
    _pivot = _bounds.getCenter()[_axis];

    for (int i = 0; i < num_objects; ++i)
    {
      // Using min and max points of the partitioning axis to find which nodes to be placed in
      double center_wrt_axis = objects[i]->getBoundingBox().getCenter()[_axis];
      // double min_wrt_axis = objects[i]->getBoundingBox().getMin()[_axis];
      // double max_wrt_axis = objects[i]->getBoundingBox().getMax()[_axis];

      if (center_wrt_axis >= _pivot)
        right_objects.push_back(objects[i]);
      else
        left_objects.push_back(objects[i]);

      // If min point is less than pivot then it should obviously be placed int the left node
      // if (min_wrt_axis < _pivot)
      // {
      //   left_objects.push_back(objects[i]);

      //   // However, the object still has the chance of being in the right node, too, if the max point is at or above the pivot
      //   if (max_wrt_axis >= _pivot)
      //     right_objects.push_back(objects[i]);
      // }
      // else
      //   right_objects.push_back(objects[i]);
    }

    if (left_objects.empty() && !right_objects.empty()) left_objects = right_objects;
    if (right_objects.empty() && !left_objects.empty()) right_objects = left_objects;

    // Add another depth level
    _left = new KdTree<T>(left_objects, depth + 1);
    _right = new KdTree<T>(right_objects, depth + 1);
  }

  ~KdTree()
  {
    if (_left)
      delete _left;
    if (_right)
      delete _right;
    if (_objects)
      delete _objects;
  }

  bool isLeaf() { return (!_left && !_right); }
  std::vector<T*> * getObjects() { return _objects; }
  double getPivot() { return _pivot; }
  int getAxis() { return _axis; }
  KdTree<T> * getLeft() { return _left; }
  KdTree<T> * getRight() { return _right; }

  bool intersect(ray& r, isect& i, bool& have_one)
  {
    double tmin;
    double tmax;

    // Do we even hit the nodes bounding box?
    if (_bounds.intersect(r, tmin, tmax))
    {
      // Recurse down tree until we hit leaf nodes
      if (!isLeaf())
      {
        bool intersected_left_node = _left->intersect(r, i, have_one);
        bool intersected_right_node = _right->intersect(r, i, have_one);

        return (intersected_left_node || intersected_right_node);
      }
      else
      {
        // See if we intersect any contained in leaf nodes
        int num_objects = _objects->size();
        for(int j = 0; j != num_objects; ++j) 
        {
          isect cur;
          if((*_objects)[j]->intersect(r, cur)) 
          {
            if(!have_one || (cur.t < i.t)) 
            {
              i = cur;
              have_one = true;
            }
          }
        }
        return have_one;
      }
    }
    else
      return false;

    // Stack-based traversal: http://www.keithlantz.net/2013/04/kd-tree-construction-using-the-surface-area-heuristic-stack-based-traversal-and-the-hyperplane-separation-theorem/
    // double tmin;
    // double tmax;
    // bool intersection_found = false;

    // // Do we even intersect with the node's bounding box?
    // if (!_bounds.intersect(r, tmin, tmax))
    //   return false;

    // stack<KDTStackElement<T>> kdt_stack;
    // KDTStackElement<T> kdt_stack_elem(this, tmin, tmax);
    // kdt_stack.push(kdt_stack_elem);

    // // Traverse kd-tree and find intersections if we have more nodes to traverse and if we haven't already found an intersection
    // while(!kdt_stack.empty() && !intersection_found)
    // {
    //   kdt_stack_elem = kdt_stack.top();
    //   kdt_stack.pop();
    //   KdTree<T> * node = kdt_stack_elem.node;
    //   tmin = kdt_stack_elem.tmin;
    //   tmax = kdt_stack_elem.tmax;

    //   // Traverse tree until we hit a leaf
    //   while (!node->isLeaf())
    //   {
    //     // Don't want to make too many method calls
    //     double pivot = node->getPivot();
    //     double axis = node->getAxis();
    //     KdTree<T> * left = node->getLeft();
    //     KdTree<T> * right = node->getRight();

    //     // Compute split intersection and designate near and far nodes
    //     double pivot_diff_origin = pivot - r.p[axis];
    //     double tstar = pivot_diff_origin / r.d[axis];
    //     bool near_is_left = (pivot_diff_origin >= 0);
    //     KdTree<T> * near = (near_is_left ? left : right);
    //     KdTree<T> * far = (near_is_left ? right : left);

    //     // Decide whether we should traverse near, far, or both
    //     if (tstar >= tmax)
    //       node = near;
    //     else if (tstar <= tmin)
    //       node = far;
    //     else
    //     {
    //       // Push far on stack for later traversal
    //       kdt_stack_elem.setParams(far, tstar, tmax);
    //       kdt_stack.push(kdt_stack_elem);
    //       node = near;
    //       tmax = tstar;
    //     }
    //   }

    //   // At leaf, find intersections with its objects
    //   vector<T*> * node_objects = node->getObjects();
    //   int num_node_objects = node_objects->size();
    //   for (int iter = 0; iter < num_node_objects; ++iter)
    //   {
    //     isect cur;
    //     if ((*node_objects)[iter]->intersect(r, cur))
    //     {
    //       if (!intersection_found || (cur.t < i.t))
    //       {
    //         i = cur;
    //         intersection_found = true;
    //       }
    //     }
    //   }

    //   // If we fell outside of the scene boundaries then we obviously haven't hit anything
    //   if (i.t > tmax)
    //     intersection_found = false;
    // }

    // return intersection_found;
  }
};

// Used for stack-based kd-tree traversal
// template <class T>
// struct KDTStackElement
// {
//   KdTree<T> * node;
//   double tmin;
//   double tmax;

//   KDTStackElement(KdTree<T> * node, double tmin, double tmax)
//   {
//     this->node = node;
//     this->tmin = tmin;
//     this->tmax = tmax;
//   }

//   void setParams(KdTree<T> * node, double tmin, double tmax)
//   {
//     this->node = node;
//     this->tmin = tmin;
//     this->tmax = tmax;
//   }
// };

class Scene {

public:
  typedef std::vector<Light*>::iterator	liter;
  typedef std::vector<Light*>::const_iterator cliter;
  typedef std::vector<Geometry*>::iterator giter;
  typedef std::vector<Geometry*>::const_iterator cgiter;

  TransformRoot transformRoot;

  Scene() : transformRoot(), objects(), lights(), kdtree(NULL) {}
  virtual ~Scene();

  void add( Geometry* obj ) {
    obj->ComputeBoundingBox();
	sceneBounds.merge(obj->getBoundingBox());
    objects.push_back(obj);
  }
  void add(Light* light) { lights.push_back(light); }

  bool intersect(ray& r, isect& i) const;

  std::vector<Light*>::const_iterator beginLights() const { return lights.begin(); }
  std::vector<Light*>::const_iterator endLights() const { return lights.end(); }

  std::vector<Geometry*>::const_iterator beginObjects() const { return objects.begin(); }
  std::vector<Geometry*>::const_iterator endObjects() const { return objects.end(); }
        
  const Camera& getCamera() const { return camera; }
  Camera& getCamera() { return camera; }

  // For efficiency reasons, we'll store texture maps in a cache
  // in the Scene.  This makes sure they get deleted when the scene
  // is destroyed.
  TextureMap* getTexture( string name );

  // These two functions are for handling ambient light; in the Phong model,
  // the "ambient" light is considered a property of the _scene_ as a whole
  // and hence should be set here.
  Vec3d ambient() const	{ return ambientIntensity; }
  void addAmbient( const Vec3d& ambient ) { ambientIntensity += ambient; }

  void glDraw(int quality, bool actualMaterials, bool actualTextures) const;

  const BoundingBox& bounds() const { return sceneBounds; }

  void buildKdTree();

 private:
  std::vector<Geometry*> objects;
  std::vector<Geometry*> nonboundedobjects;
  std::vector<Geometry*> boundedobjects;
  std::vector<Light*> lights;
  Camera camera;

  // This is the total amount of ambient light in the scene
  // (used as the I_a in the Phong shading model)
  Vec3d ambientIntensity;

  typedef std::map< std::string, TextureMap* > tmap;
  tmap textureCache;
	
  // Each object in the scene, provided that it has hasBoundingBoxCapability(),
  // must fall within this bounding box.  Objects that don't have hasBoundingBoxCapability()
  // are exempt from this requirement.
  BoundingBox sceneBounds;
  
  KdTree<Geometry> * kdtree;

 public:
  // This is used for debugging purposes only.
  mutable std::vector<std::pair<ray*, isect*> > intersectCache;
};

#endif // __SCENE_H__
