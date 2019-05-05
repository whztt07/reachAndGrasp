#pragma once
#include "Ogre.h"

static bool IntersectionLineAABB(Ogre::Vector3 lineStartPos, Ogre::Vector3 lineEndPos, Ogre::AxisAlignedBox &b)
{
	Ogre::Vector3 p0 = lineStartPos;
	Ogre::Vector3 p1 = lineEndPos;

	Ogre::Vector3 c = (b.getMinimum() + b.getMaximum()) * 0.5f; // Box center-point
	Ogre::Vector3 e = b.getMaximum() - c;             // Box halflength extents
	Ogre::Vector3 m = (p0 + p1) * 0.5f;       // Segment midpoint
	Ogre::Vector3 d = p1 - m;                // Segment halflength vector
	m = m - c;                        // Translate box and segment to origin
	
	// Try world coordinate axes as separating axes
	float adx = abs(d.x);
	if (abs(m.x) > e.x + adx) return 0;
	float ady = abs(d.y);
	if (abs(m.y) > e.y + ady) return 0;
	float adz = abs(d.z);
	if (abs(m.z) > e.z + adz) return 0;
	
	// Add in an epsilon term to counteract arithmetic errors when segment is
	// (near) parallel to a coordinate axis (see text for detail)
	float epslion = std::numeric_limits<float>::epsilon();
	adx += epslion; ady += epslion; adz += epslion;
	
	// Try cross products of segment direction vector with coordinate axes
	if (abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return 0;
	if (abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return 0;
	if (abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return 0;
	
	// No separating axis found; segment must be overlapping AABB
	return 1;
}

static bool IntersectionLineSphere(Ogre::Vector3 lineStartPos, Ogre::Vector3 lineEndPos, Ogre::Sphere &sphere)
{
	Ogre::Vector3 p = lineStartPos;
	Ogre::Vector3 d = (lineEndPos - lineStartPos);
	float length = (lineEndPos - lineStartPos).length();
	d.normalise();
	Ogre::Vector3 m = p - sphere.getCenter();
	float aa = m.length();

	float b = m.dotProduct(d);// Dot(m, d);
	float c = m.dotProduct(m) - sphere.getRadius() * sphere.getRadius();
	// Exit if r抯 origin outside s (c > 0)and r pointing away from s (b > 0)
	if (c > 0.0f && b > 0.0f) return false;
	float discr = b*b - c;
	// A negative discriminant corresponds to ray missing sphere
	if (discr < 0.0f) return false;

	//to this point， ray hits sphere 
    // check if intersection inside the line segment
	//intersection0
	float t = -b + sqrt(discr);
	if (t >= 0 && t <= length) return true;
	//intersection1
	      t = -b - sqrt(discr);
	if (t >= 0 && t <= length) return true;

	return false;
}

static bool IntersectionLine(Ogre::Vector3 lineStartPos, Ogre::Vector3 lineEndPos, std::vector<Ogre::Sphere> &spheres, std::vector<Ogre::AxisAlignedBox> &bs)
{
	//spheres
	for (int i = 0; i < spheres.size(); i++)
	{
		if (IntersectionLineSphere(lineStartPos, lineEndPos, spheres[i]))
			return true;
	}
	
	//boxes
	for (int i = 0; i < bs.size(); i++)
	{
		if (IntersectionLineAABB(lineStartPos, lineEndPos, bs[i]))
			return true;
	}

	return false;
}


//
//
////== = Section 5.3.2: == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =
//
//// Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting,
//// returns t value of intersection and intersection point q
//int IntersectRaySphere(Point p, Vector d, Sphere s, float &t, Point &q)
//{
//	Vector m = p ? s.c;
//	float b = Dot(m, d);
//	float c = Dot(m, m) ? s.r * s.r;
//	// Exit if r抯 origin outside s (c > 0)and r pointing away from s (b > 0)
//	if (c > 0.0f && b > 0.0f) return 0;
//	float discr = b*b ? c;
//	// A negative discriminant corresponds to ray missing sphere
//	if (discr < 0.0f) return 0;
//	// Ray now found to intersect sphere, compute smallest t value of intersection
//	t = -b ? Sqrt(discr);
//	// If t is negative, ray started inside sphere so clamp t to zero
//	if (t < 0.0f) t = 0.0f;
//	q = p + t * d;
//	return 1;
//}
//
////-------------------------------------------------------------------------------
//
//// Test if ray r = p + td intersects sphere s
//int TestRaySphere(Point p, Vector d, Sphere s)
//{
//	Vector m = p ? s.c;
//	float c = Dot(m, m) ? s.r * s.r;
//	// If there is definitely at least one real root, there must be an intersection
//	if (c <= 0.0f) return 1;
//	float b = Dot(m, d);
//	// Early exit if ray origin outside sphere and ray pointing away from sphere
//	if (b > 0.0f) return 0;
//	float disc = b*b ? c;
//	// A negative discriminant corresponds to ray missing sphere
//	if (disc < 0.0f) return 0;
//	// Now ray must hit sphere
//	return 1;
//}
//
//
////== = Section 5.3.3: == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =
//
//// Intersect ray R(t) = p + t*d against AABB a. When intersecting,
//// return intersection distance tmin and point q of intersection
//int IntersectRayAABB(Point p, Vector d, AABB a, float &tmin, Point &q)
//{
//	tmin = 0.0f;          // set to -FLT_MAX to get first hit on line
//	float tmax = FLT_MAX; // set to max distance ray can travel (for segment)
//
//	// For all three slabs
//	for (int i = 0; i < 3; i++) {
//		if (Abs(d[i]) < EPSILON) {
//			// Ray is parallel to slab. No hit if origin not within slab
//			if (p[i] < a.min[i] || p[i] > a.max[i]) return 0;
//		}
//		else {
//			// Compute intersection t value of ray with near and far plane of slab
//			float ood = 1.0f / d[i];
//			float t1 = (a.min[i] - p[i]) * ood;
//			float t2 = (a.max[i] - p[i]) * ood;
//			// Make t1 be intersection with near plane, t2 with far plane
//			if (t1 > t2) Swap(t1, t2);
//			// Compute the intersection of slab intersections intervals
//			tmin = Max(tmin, t1);
//			tmax = Min(tmax, t2);
//			// Exit with no collision as soon as slab intersection becomes empty
//			if (tmin > tmax) return 0;
//		}
//	}
//	// Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin) 
//	q = p + d * tmin;
//	return 1;
//}
//
////--------------------------------------------------------------------------------
//
//// Test if segment specified by points p0 and p1 intersects AABB b
//int TestSegmentAABB(Point p0, Point p1, AABB b)
//{
//	Point c = (b.min + b.max) * 0.5f; // Box center-point
//	Vector e = b.max - c;             // Box halflength extents
//	Point m = (p0 + p1) * 0.5f;       // Segment midpoint
//	Vector d = p1 - m;                // Segment halflength vector
//	m = m - c;                        // Translate box and segment to origin
//
//	// Try world coordinate axes as separating axes
//	float adx = Abs(d.x);
//	if (Abs(m.x) > e.x + adx) return 0;
//	float ady = Abs(d.y);
//	if (Abs(m.y) > e.y + ady) return 0;
//	float adz = Abs(d.z);
//	if (Abs(m.z) > e.z + adz) return 0;
//
//	// Add in an epsilon term to counteract arithmetic errors when segment is
//	// (near) parallel to a coordinate axis (see text for detail)
//	adx += EPSILON; ady += EPSILON; adz += EPSILON;
//
//	// Try cross products of segment direction vector with coordinate axes
//	if (Abs(m.y * d.z - m.z * d.y) > e.y * adz + e.z * ady) return 0;
//	if (Abs(m.z * d.x - m.x * d.z) > e.x * adz + e.z * adx) return 0;
//	if (Abs(m.x * d.y - m.y * d.x) > e.x * ady + e.y * adx) return 0;
//
//	// No separating axis found; segment must be overlapping AABB
//	return 1;
//}
//
////--------------------------------------------------------------------------------
//
//Vector e = b.max ? b.min;
//Vector d = p1 - p0;
//Point m = p0 + p1 - b.min - b.max;
//
//
//
//
//
////== = Section 5.3.7: == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =
//
//// Intersect segment S(t)=sa+t(sb-sa), 0<=t<=1 against cylinder specified by p, q and r
//int IntersectSegmentCylinder(Point sa, Point sb, Point p, Point q, float r, float &t)
//{
//	Vector d = q ? p, m = sa ? p, n = sb ? sa;
//	float md = Dot(m, d);
//	float nd = Dot(n, d);
//	float dd = Dot(d, d);
//	// Test if segment fully outside either endcap of cylinder
//	if (md < 0.0f && md + nd < 0.0f) return 0; // Segment outside 憄?side of cylinder
//	if (md > dd && md + nd > dd) return 0;     // Segment outside 憅?side of cylinder
//	float nn = Dot(n, n);
//	float mn = Dot(m, n);
//	float a = dd * nn ? nd * nd;
//	float k = Dot(m, m) ? r * r;
//	float c = dd * k ? md * md;
//	if (Abs(a) < EPSILON) {
//		// Segment runs parallel to cylinder axis
//		if (c > 0.0f) return 0; // 慳?and thus the segment lie outside cylinder
//		// Now known that segment intersects cylinder; figure out how it intersects
//		if (md < 0.0f) t = -mn / nn; // Intersect segment against 憄?endcap
//		else if (md > dd) t = (nd - mn) / nn; // Intersect segment against 憅?endcap
//		else t = 0.0f; // 慳?lies inside cylinder
//		return 1;
//	}
//	float b = dd * mn ? nd * md;
//	float discr = b * b ? a * c;
//	if (discr < 0.0f) return 0; // No real roots; no intersection
//	t = (-b ? Sqrt(discr)) / a;
//	if (t < 0.0f || t > 1.0f) return 0; // Intersection lies outside segment
//	if (md + t * nd < 0.0f) {
//		// Intersection outside cylinder on 憄?side
//		if (nd <= 0.0f) return 0; // Segment pointing away from endcap
//		t = -md / nd;
//		// Keep intersection if Dot(S(t) - p, S(t) - p) <= r^2
//		return k + 2 * t * (mn + t * nn) <= 0.0f;
//	}
//	else if (md + t * nd > dd) {
//		// Intersection outside cylinder on 憅?side
//		if (nd >= 0.0f) return 0; // Segment pointing away from endcap
//		t = (dd ? md) / nd;
//		// Keep intersection if Dot(S(t) - q, S(t) - q) <= r^2
//		return k + dd ? 2 * md + t * (2 * (mn ? nd) + t * nn) <= 0.0f;
//	}
//	// Segment intersects cylinder between the end-caps; t is correct
//	return 1;
//}
//
