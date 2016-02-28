#include "vector_math.h"
#include <vxWorks.h>
#include <math.h>
#include "harlielog.h"

Point2d zero;

// Always returns false?
int h_isnan(float x) 
{
    return x != x;
}

// Constructing a vector from the two 2D points
STATUS vecFromPoint2ds(const Point2d * p1, const Point2d * p2, Vector2d * const v) 
{
    v->p.x = p2->x - p1->x;
    v->p.y = p2->y - p1->y;
    return (OK);
}

// Returns a zero vector because only one point was gven
STATUS vecFromPoint2d(const Point2d * p1, Vector2d * const v) 
{
    zero.x = 0.0;
    zero.y = 0.0;

    return vecFromPoint2ds(&zero, p1, v);
}

// Returns the distance between the two points
float distance(const Point2d *p1, const Point2d *p2) 
{
    float temp = p2->x - p1->x;
    float temp2 = p2->y - p1->y;

    temp = (float) pow(temp, 2.0) + (float) pow(temp2, 2.0);
    temp = (float) sqrt(temp);

    return temp;
}

// Calculates the dot product of two given vectors
float dot(const Vector2d *v1, const Vector2d *v2) 
{
    float sum = v1->p.x * v2->p.x + v1->p.y * v2->p.y;
    return sum;
}

// Calculates the normal vector of the given vector
STATUS normalize(const Vector2d * v, Vector2d * const output) 
{
    LOG.VERBOSE("v.x: %f, v.y: %f", v->p.x, v->p.y);
    float length_sq = dot(v, v);
    LOG.VERBOSE("length_squared: %f", length_sq);
    float length = (float) sqrt((double) length_sq);
    LOG.VERBOSE("length: %f", length);
    output->p.x = v->p.x / length;
    output->p.y = v->p.y / length;
    return (OK);
}

// Subtracts two vectors v1 - v2
STATUS sub(const Vector2d *v1, const Vector2d *v2, Vector2d * const output) {
/*
 * Computes output = v1 - v2
*/
    output->p.x = v1->p.x - v2->p.x;
    output->p.y = v1->p.y - v2->p.y;
    return (OK);
}

