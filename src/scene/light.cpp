#include <cmath>

#include "light.h"

using namespace std;

double DirectionalLight::distanceAttenuation(const Vec3d& P) const
{
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.

  // Lighting model equation: http://www.cs.utexas.edu/~fussell/courses/cs354/assignments/raytracing/equations.pdf
  ray point_to_light(p, -orientation,  ray::SHADOW);
  isect intersect_info;

  // See if the ray from the point to the light intersects with any object, if it doesn't then just return the light color
  if (scene->intersect(point_to_light, intersect_info))
  {
    // We have an intersection
      
  }
  else
    return color;
}

Vec3d DirectionalLight::getColor() const
{
  return color;
}

Vec3d DirectionalLight::getDirection(const Vec3d& P) const
{
  // for directional light, direction doesn't depend on P
  return -orientation;
}

double PointLight::distanceAttenuation(const Vec3d& P) const
{

  // YOUR CODE HERE

  // You'll need to modify this method to attenuate the intensity 
  // of the light based on the distance between the source and the 
  // point P.  For now, we assume no attenuation and just return 1.0

  // Base on slide 19: http://www.cs.utexas.edu/~fussell/courses/cs354/lectures/lecture10.pdf
  double distance = (P - position).length();
  double attenuation = 1.0 / (constantTerm + (linearTerm * distance) + (quadraticeTerm * distance * distance));

  // Light intensity should only get weaker or stay the same
  return min(1.0, attenuation);
}

Vec3d PointLight::getColor() const
{
  return color;
}

Vec3d PointLight::getDirection(const Vec3d& P) const
{
  Vec3d ret = position - P;
  ret.normalize();
  return ret;
}


Vec3d PointLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.

  // Lighting model equation: http://www.cs.utexas.edu/~fussell/courses/cs354/assignments/raytracing/equations.pdf
  ray point_to_light(p, getDirection(p));
  isect intersect_info;

  // See if the ray from the point to the light intersects with any object
  if (scene->intersect(point_to_light, intersect_info))
  {  
  }
  else
    return color;
}
