#include "shapes/snowmanex.h"

#include "efloat.h"
#include "paramset.h"
#include "sampling.h"
#include "shapes/cone.h"
#include "shapes/sphere.h"
#include "stats.h"
// #include <iostream> // for print

namespace pbrt {
STAT_COUNTER("[Step 4]/The number of rays that hit the body",
             nRayIntersectionBody);
STAT_COUNTER("[Step 4]/The number of rays that hit the head",
             nRayIntersectionHead);
STAT_COUNTER("[Step 4]/The number of rays that hit the hat",
             nRayIntersectionHat);
STAT_COUNTER("[Step 4]/The number of rays that hit the nose",
             nRayIntersectionNose);

// SnowManEX Method Definitions
SnowManEX::SnowManEX(const Transform *ObjectToWorld,
                     const Transform *WorldToObject, bool reverseOrientation,
                     Float radiusHead, Float radiusBody, Float radiusHat,
                     Float heightHat, Point3f &posHead, Point3f &posBody,
                     Point3f &posHat, Float phiMax)
    : Shape(ObjectToWorld, WorldToObject, reverseOrientation),
      radiusHead(radiusHead),
      radiusBody(radiusBody),
      radiusHat(radiusHat),
      heightHat(heightHat),
      posHead(posHead),
      posBody(posBody),
      posHat(posHat),
      phiMax(Radians(Clamp(phiMax, 0, 360))),
      object2worldHead((*ObjectToWorld) *
                       Translate(Vector3f(posHead.x, posHead.y, posHead.z))),
      world2objectHead(
          Inverse(Translate(Vector3f(posHead.x, posHead.y, posHead.z))) *
          (*WorldToObject)),
      object2worldNose((*ObjectToWorld) *
                       Translate(Vector3f(posHead.x, posHead.y, posHead.z)) *
                       Rotate(90, Vector3f(0, 1, 0))),
      world2objectNose(
          Inverse(Rotate(90, Vector3f(0, 1, 0))) *
          Inverse(Translate(Vector3f(posHead.x, posHead.y, posHead.z))) *
          (*WorldToObject)),
      object2worldBody((*ObjectToWorld) *
                       Translate(Vector3f(posBody.x, posBody.y, posBody.z))),
      world2objectBody(
          (*WorldToObject) *
          Inverse(Translate(Vector3f(posBody.x, posBody.y, posBody.z)))),
      object2worldHat((*ObjectToWorld) *
                      Translate(Vector3f(posHat.x, posHat.y, posHat.z))),
      world2objectHat(
          Inverse(Translate(Vector3f(posHat.x, posHat.y, posHat.z))) *
          (*WorldToObject)),
      sphereHead(&object2worldHead, &world2objectHead, false, radiusHead,
                 -radiusHead, radiusHead, phiMax),  // 360.f
      sphereBody(&object2worldBody, &world2objectBody, false, radiusBody,
                 -radiusBody, radiusBody, phiMax),
      coneHat(&object2worldHat, &world2objectHat, false, heightHat, radiusHat,
              phiMax),
      cylinderNose(&object2worldNose, &world2objectNose, false, radiusHead / 5,
                   0, radiusHead * 1.5, phiMax) {}

// Bounds3f SnowManEX::ObjectBound() const {
//     Point3f pmin, pmax;
//     pmin.x = std::min(
//         {posHead.x - radiusHead, posBody.x - radiusBody, posHat.x -
//         radiusHat});
//     pmax.x = std::max(
//         {posHead.x + static_cast<float>(radiusHead*1.5), posBody.x +
//         radiusBody, posHat.x + radiusHat});
//     pmin.y = std::min(
//         {posHead.y - radiusHead, posBody.y - radiusBody, posHat.y -
//         radiusHat, -radiusHead/5});
//     pmax.y = std::max(
//         {posHead.y + radiusHead, posBody.y + radiusBody, posHat.y +
//         radiusHat, radiusHead/5});
//     pmin.z =
//         std::min({posHead.z - radiusHead, posBody.z - radiusBody, posHat.z,
//         static_cast<float>(0)});
//     pmax.z = std::max(
//         {posHead.z + radiusHead, posBody.z + radiusBody, posHat.z +
//         heightHat, radiusHead*2});

//     return Bounds3f(pmin, pmax);
// }

// Bounds3f SnowManEX::ObjectBound() const {
//     Point3f pmin, pmax;
//     pmin.x = -100;
//     pmax.x = 100;
//     pmin.y = -100;
//     pmax.y = 100;
//     pmin.z = -100;
//     pmax.z = 100;

//     return Bounds3f(pmin, pmax);
// }

Bounds3f SnowManEX::ObjectBound() const {
    Bounds3f headbbox = object2worldHead(sphereHead.ObjectBound());
    Bounds3f bodybbox = object2worldBody(sphereBody.ObjectBound());
    Bounds3f hatbbox = object2worldHat(coneHat.ObjectBound());
    Bounds3f nosebbox = object2worldNose(cylinderNose.ObjectBound());
    // std::cout << "Head: " << headbbox.pMax << "; " << headbbox.pMin
    //           << " --------------------------------\n";
    // std::cout << "Nose: " << nosebbox.pMax << "; " << nosebbox.pMin
    //           << " --------------------------------\n";
    Bounds3f bbox = Union(headbbox, bodybbox);
    bbox = Union(bbox, hatbbox);
    bbox = Union(bbox, nosebbox);
    bbox.pMax.x += 1;
    bbox.pMax.y += 1;
    bbox.pMax.z += 1;
    bbox.pMin.x -= 1;
    bbox.pMin.y -= 1;
    bbox.pMin.z -= 1;
    return bbox;
}

bool SnowManEX::Intersect(const Ray &r, Float *tHit, SurfaceInteraction *isect,
                          bool testAlphaTexture) const {
    ProfilePhase p(Prof::ShapeIntersect);

    Float closestT = std::numeric_limits<Float>::infinity();
    bool hit = false;
    int64_t *objcounter = nullptr;

    std::vector<std::tuple<const Shape *, int64_t *, std::string, int>>
        objects = {{&sphereBody, &nRayIntersectionBody, "body", 0},
                   {&sphereHead, &nRayIntersectionHead, "head", 1},
                   {&cylinderNose, &nRayIntersectionNose, "nose", 2},
                   {&coneHat, &nRayIntersectionHat, "hat", 3}};

    for (const auto obj : objects) {
        Float tHitTmp;
        SurfaceInteraction isectTmp;
        isectTmp.partName = &std::get<2>(obj);
        isectTmp.partID = std::get<3>(obj);

        if (std::get<0>(obj)->Intersect(r, &tHitTmp, &isectTmp,
                                        testAlphaTexture)) {
            if (tHitTmp < closestT) {
                closestT = tHitTmp;
                *isect = isectTmp;
                isect->partID = std::get<3>(obj);
                isect->partName = &std::get<2>(obj);
                hit = true;
                objcounter = std::get<1>(obj);
            }
        }
    }

    if (hit) {
        // std::cout << "\npartID: " << isect->partID << "\n";
        *tHit = closestT;
        ++(*objcounter);
    }

    return hit;
}

bool SnowManEX::IntersectP(const Ray &r, bool testAlphaTexture) const {
    ProfilePhase p(Prof::ShapeIntersectP);

    if (sphereHead.IntersectP(r, testAlphaTexture) ||
        sphereBody.IntersectP(r, testAlphaTexture) ||
        coneHat.IntersectP(r, testAlphaTexture) ||
        cylinderNose.IntersectP(r, testAlphaTexture))
        return true;

    return false;
}

Float SnowManEX::Area() const {
    return sphereHead.Area() + sphereBody.Area() + coneHat.Area() +
           cylinderNose.Area();
}

Interaction SnowManEX::Sample(const Point2f &u, Float *pdf) const {
    LOG(FATAL) << "SnowManEX::Sample not implemented.";
    return Interaction();
}

std::shared_ptr<Shape> CreateSnowManEXShape(const Transform *o2w,
                                            const Transform *w2o,
                                            bool reverseOrientation,
                                            const ParamSet &params) {
    Float radiusHead = params.FindOneFloat("radiusHead", 0.5f);
    Float radiusBody = params.FindOneFloat("radiusBody", 1.f);
    Float radiusHat = params.FindOneFloat("radiusHat", .5f);
    Float heightHat = params.FindOneFloat("heightHat", 1.f);
    Point3f posHead = params.FindOnePoint3f("posHead", Point3f(0, 0, 1.1));
    Point3f posBody = params.FindOnePoint3f("posBody", Point3f(0, 0, 0));
    Point3f posHat = params.FindOnePoint3f("posHat", Point3f(0, 0, 2.1));
    Float phimax = params.FindOneFloat("phimax", 360.f);
    return std::make_shared<SnowManEX>(o2w, w2o, reverseOrientation, radiusHead,
                                       radiusBody, radiusHat, heightHat,
                                       posHead, posBody, posHat, phimax);
}

}  // namespace pbrt
