#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const Vec3d &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const Vec3d &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    // scene->add(newFace);
    return true;
}

char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	double tmin = 0.0;
	double tmax = 0.0;
	typedef Faces::const_iterator iter;
	bool have_one = false;
	for( iter j = faces.begin(); j != faces.end(); ++j )
	  {
	    isect cur;
	    if( (*j)->intersectLocal( r, cur ) )
	      {
		if( !have_one || (cur.t < i.t) )
		  {
		    i = cur;
		    have_one = true;
		  }
	      }
	  }
	if( !have_one ) i.setT(1000.0);
	return have_one;
}

bool TrimeshFace::intersect(ray& r, isect& i) const {
  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{

    const Vec3d& a = parent->vertices[ids[0]];
    const Vec3d& b = parent->vertices[ids[1]];
    const Vec3d& c = parent->vertices[ids[2]];

    // YOUR CODE HERE

    // Following code is based on this article: http://geomalgorithms.com/a06-_intersect-2.html
    // First, check to see if we intersect with the plane that the triangle resides in
    double t_denom =  normal * (r.d);

    // The ray is parallel to the plane (infinite/no intersection)
    if (abs(t_denom) < RAY_EPSILON)
        return false;

    // Compute the t value at which the ray intersects the plane
    double t = (normal * (a - r.p)) / t_denom;

    // Behind us
    if (t < 0.0)
        return false;

    // We intersect the plane, now find if the point intersects with the triangle
    Vec3d u = b - a;
    Vec3d v = c - a;
    Vec3d w = ((r.p + r.d * t) - a);
    double u_dot_v = u * v;
    double u_dot_u = u.length2();
    double v_dot_v = v.length2();
    double w_dot_v = w * v;
    double w_dot_u = w * u;

    // Compute twice the triangle's area (parallelogram)
    double bary_denom = (u_dot_v * u_dot_v) - (u_dot_u * v_dot_v);

    // Unlikely for this to happen, but an area of zero implies that the ray never intersects
    if (abs(bary_denom) < RAY_EPSILON)
        return false;

    // Compute barycentric coordinates of intersection point
    double beta = ((u_dot_v * w_dot_v) - (v_dot_v * w_dot_u)) / bary_denom;
    double gamma = ((u_dot_v * w_dot_u) - (u_dot_u * w_dot_v)) / bary_denom;
    double alpha = 1.0 - (alpha + beta);

    // If any of the barycentric coordinates are less than 0 then we did not intersect the triangle
    if (alpha < 0.0 || beta < 0.0 || gamma < 0.0)
        return false;

    // We have an intersection!
    // Set t value and barycentric coordinates
    i.t = t;
    i.bary[0] = alpha;
    i.bary[1] = beta;
    i.bary[2] = gamma;

    // Interpolate vertex normals to find normal at point or just take surface normal
    if (parent->vertNorms)
    {
        const Vec3d& na = parent->normals[ids[0]];
        const Vec3d& nb = parent->normals[ids[1]];
        const Vec3d& nc = parent->normals[ids[2]];

        // Barycentric interpolation is some cool shit
        i.N = ((alpha * na) + (beta * nb) + (gamma * nc)).normalize(); 
    }
    else
        i.N = normal.normalize();

    return true;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		Vec3d faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}
