/* 
 * File:   vector_math.h
 * Author: Eric Perko
 *
 * Created on October 30, 2009, 1:55 PM
 */

#include <vxWorks.h>

#ifndef _VECTOR_MATH_H
#define	_VECTOR_MATH_H

#ifdef	__cplusplus
extern "C" {
#endif

    // 2 dimensional struct
    typedef struct Point2d_t 
    {
        float x;
        float y;
    } Point2d;

    // 2 dimensional vector
    typedef struct Vector2d_t 
    {
        // Origin is assumed to be point (0,0) 
        Point2d p;
    } Vector2d;

    // Always returns false?
    int h_isnan(float x);
    // Construcs a vector form two 2D points
    STATUS vecFromPoint2ds(const Point2d * p1, const Point2d * p2, Vector2d * const v);
    // Returns a zero vector because only one point was given
    STATUS vecFromPoint2d(const Point2d * p1, Vector2d * const v);
    // Calculates the dot product of the two given vectors 
    float dot(const Vector2d *v1, const Vector2d *v2);
    // Caluclates the distance between two 2D points
    float distance(const Point2d *p1, const Point2d *p2);
    // Caculates the normal vector of the given vector
    STATUS normalize(const Vector2d * v, Vector2d * const output);
    // Subtracts v1 from v2 
    STATUS sub(const Vector2d *v1, const Vector2d *v2, Vector2d * const output);

#ifdef	__cplusplus
}
#endif

#endif	/* _VECTOR_MATH_H */
