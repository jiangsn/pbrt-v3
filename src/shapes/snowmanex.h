#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_SHAPES_SNOWMANEX_H
#define PBRT_SHAPES_SNOWMANEX_H

#include "../core/transform.h"
#include "cone.h"
#include "cylinder.h"
#include "shape.h"
#include "sphere.h"

namespace pbrt {

// SnowManEX Declarations
class SnowManEX : public Shape {
  public:
    // SnowManEX Public Methods
    SnowManEX(const Transform *ObjectToWorld, const Transform *WorldToObject,
              bool reverseOrientation, Float radiusHead, Float radiusBody,
              Float radiusHat, Float heightHat, Point3f &posHead,
              Point3f &posBody, Point3f &posHat, Float phiMax);
    Bounds3f ObjectBound() const;
    bool Intersect(const Ray &ray, Float *tHit, SurfaceInteraction *isect,
                   bool testAlphaTexture) const;
    bool IntersectP(const Ray &ray, bool testAlphaTexture) const;
    Interaction Sample(const Point2f &u, Float *pdf) const;
    Float Area() const;

  private:
    // SnowManEX Private Data
    const Float radiusHead, radiusBody, radiusHat, heightHat;
    const Float phiMax;
    const Point3f posHead, posBody, posHat;

    const Transform identity;
    const Transform object2worldHead, object2worldBody, object2worldHat,
        object2worldNose;
    const Transform world2objectHead, world2objectBody, world2objectHat,
        world2objectNose;
    Sphere sphereHead, sphereBody;
    Cone coneHat;
    Cylinder cylinderNose;
};

std::shared_ptr<Shape> CreateSnowManEXShape(const Transform *o2w,
                                            const Transform *w2o,
                                            bool reverseOrientation,
                                            const ParamSet &params);

}  // namespace pbrt

#endif  // PBRT_SHAPES_SNOWMANEX_H
