/*
* Assignment #6
* Name: Jason Palacios
* UT EID: jap4839
* UTCS: jason777
*/

#include <cmath>

#include "light.h"
#include <iostream>

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
  ray point_to_light(p, -orientation, ray::SHADOW);
  isect intersect_info;

  // See if the ray from the point to the light intersects with any object, if it doesn't then just return the light color
  if (scene->intersect(point_to_light, intersect_info))
  {
    // Grab the point of intersection
    Vec3d q = point_to_light.at(intersect_info.t);

    // Grab transmissive material property
    Vec3d kt = intersect_info.material->kt(intersect_info);

    // Find length between p and q to use for dropoff of shadow color (only useful whenever we have a transparent object and treat q as a new point light source)
    // double distance_pq = (p - q).length();
    // double attenuation = min(1.0, 1.0 / (1.0 + (0.1 * distance_pq) + (0.01 * distance_pq * distance_pq)));

    // We might intersect with another object using the ray from q to the light source
    Vec3d intensity_at_q = shadowAttenuation(r, q);

    // Return the componenet-wise multiplication of transmissive material property and color at q
    return prod(intensity_at_q, kt);
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
  double attenuation = 1.0 / (constantTerm + (linearTerm * distance) + (quadraticTerm * distance * distance));

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
  ray point_to_light(p, getDirection(p), ray::SHADOW);
  isect intersect_info;

  // Find intersection point
  bool intersection = scene->intersect(point_to_light, intersect_info);
  Vec3d q = point_to_light.at(intersect_info.t);

  // Find length between p and q to use for dropoff of shadow color 
  double distance_pq = (p - q).length();
  bool before_light = (distance_pq < (position - p).length());

  // See if the ray from the point to the light intersects with any object, if it doesn't then just return the light color
  if (intersection && before_light)
  {
    // Grab transmissive material property
    Vec3d kt = intersect_info.material->kt(intersect_info);

    // Compute dropoff coefficient (only useful whenever we have a transparent object and treat q as a new light source)
    // double attenuation = min(1.0, 1.0 / (constantTerm + (linearTerm * distance_pq) + (quadraticTerm * distance_pq * distance_pq)));

    // We might intersect with another object using the ray from q to the light source
    Vec3d intensity_at_q = shadowAttenuation(r, q);

    // Return the componenet-wise multiplication of transmissive material property and color at q
    return prod(intensity_at_q, kt);
  }
  else
    return color;
}
