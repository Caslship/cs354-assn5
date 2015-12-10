#ifndef TRIMESH_H__
#define TRIMESH_H__

#include <list>
#include <vector>

#include "../scene/ray.h"
#include "../scene/material.h"
#include "../scene/scene.h"

class TrimeshFace;

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

public:
    Trimesh( Scene *scene, Material *mat, TransformNode *transform )
        : MaterialSceneObject(scene, mat), 
			displayListWithMaterials(0),
			displayListWithoutMaterials(0)
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

    bool isTrimesh() const { return true; }
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

class TrimeshFace : public MaterialSceneObject
{
    Trimesh *parent;
    int ids[3];
    Vec3d normal;
    double dist;
    Vec3d u;
    Vec3d v;
    double u_dot_u;
    double v_dot_v;
    double u_dot_v;
    double tri_area;
    bool zero_area_flag;

public:
    TrimeshFace( Scene *scene, Material *mat, Trimesh *parent, int a, int b, int c)
        : MaterialSceneObject( scene, mat )
    {
        this->parent = parent;
        ids[0] = a;
        ids[1] = b;
        ids[2] = c;

		// Compute the face normal here, not on the fly
		Vec3d a_coords = parent->vertices[a];
		Vec3d b_coords = parent->vertices[b];
		Vec3d c_coords = parent->vertices[c];

		Vec3d vab = (b_coords - a_coords);
		Vec3d vac = (c_coords - a_coords);
		Vec3d vcb = (b_coords - c_coords);

        // Precompute these values for ray-triangle intersection
        u = vab;
        v = vac;
        u_dot_v = u * v;
        u_dot_u = u.length2();
        v_dot_v = v.length2();
        tri_area = (u_dot_v * u_dot_v) - (u_dot_u * v_dot_v);
        zero_area_flag = (abs(tri_area) < RAY_EPSILON);
        
        // Compute normal
		if (vab.iszero() || vac.iszero() || vcb.iszero()) degen = true;
		else {
			degen = false;
			normal = ((b_coords - a_coords) ^ (c_coords - a_coords));
			normal.normalize();
			dist = normal * a_coords;
		}
		localbounds = ComputeLocalBoundingBox();
		bounds = localbounds;
    }

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
      
    BoundingBox ComputeLocalBoundingBox()
    {
        BoundingBox localbounds;
        localbounds.setMax(maximum( parent->vertices[ids[0]], parent->vertices[ids[1]]));
		localbounds.setMin(minimum( parent->vertices[ids[0]], parent->vertices[ids[1]]));
        
        localbounds.setMax(maximum( parent->vertices[ids[2]], localbounds.getMax()));
		localbounds.setMin(minimum( parent->vertices[ids[2]], localbounds.getMin()));
        return localbounds;
    }

    const BoundingBox& getBoundingBox() const { return localbounds; }

 };

#endif // TRIMESH_H__
