#ifndef TRIMESH_H__
#define TRIMESH_H__

#include <list>
#include <vector>

#include "../scene/ray.h"
#include "../scene/material.h"
#include "../scene/scene.h"

class Trimesh;

class TrimeshFace : public MaterialSceneObject
{
    Trimesh *parent;
    int ids[3];
    Vec3d normal;
    double dist;

public:
    TrimeshFace( Scene *scene, Material *mat, Trimesh *parent, int a, int b, int c);

    BoundingBox localbounds;
    bool degen;

    int operator[]( int i ) const
    {
        return ids[i];
    }

    Vec3d getNormal()
    {
        return normal;
    }

    bool intersect(ray& r, isect& i ) const;
    bool intersectLocal(ray& r, isect& i ) const;

    bool hasBoundingBoxCapability() const { return true; }
      
    BoundingBox ComputeLocalBoundingBox();

    const BoundingBox& getBoundingBox() const { return localbounds; }

 };

class Trimesh : public MaterialSceneObject
{
    friend class TrimeshFace;
    typedef std::vector<Vec3d> Normals;
    typedef std::vector<Vec3d> Vertices;
    typedef std::vector<TrimeshFace*> Faces;
    typedef std::vector<Material*> Materials;

    Vertices vertices;
    Faces faces;
    Normals normals;
    Materials materials;
	BoundingBox localBounds;

    KdTree<TrimeshFace> * kdtree;

public:
    Trimesh( Scene *scene, Material *mat, TransformNode *transform )
        : MaterialSceneObject(scene, mat), 
			displayListWithMaterials(0),
			displayListWithoutMaterials(0), kdtree(NULL)
    {
      this->transform = transform;
      vertNorms = false;
    }

    bool vertNorms;

    bool intersectLocal(ray& r, isect& i) const;

    ~Trimesh();
    
    // must add vertices, normals, and materials IN ORDER
    void addVertex( const Vec3d & );
    void addMaterial( Material *m );
    void addNormal( const Vec3d & );
    bool addFace( int a, int b, int c );

    char *doubleCheck();
    
    void generateNormals();

    virtual bool isTrimesh() const { return true; }
    virtual void buildKdTree()
    {
        if (kdtree)
            delete kdtree;

        kdtree = new KdTree<TrimeshFace>(faces, 0);
    }

    bool hasBoundingBoxCapability() const { return true; }
      
    BoundingBox ComputeLocalBoundingBox()
    {
        BoundingBox localbounds;
		if (vertices.size() == 0) return localbounds;
		localbounds.setMax(vertices[0]);
		localbounds.setMin(vertices[0]);
		Vertices::const_iterator viter;
		for (viter = vertices.begin(); viter != vertices.end(); ++viter)
	  {
	    localbounds.setMax(maximum( localbounds.getMax(), *viter));
	    localbounds.setMin(minimum( localbounds.getMin(), *viter));
	  }
		localBounds = localbounds;
        return localbounds;
    }

protected:
	void glDrawLocal(int quality, bool actualMaterials, bool actualTextures) const;
	mutable int displayListWithMaterials;
	mutable int displayListWithoutMaterials;
};

#endif // TRIMESH_H__
