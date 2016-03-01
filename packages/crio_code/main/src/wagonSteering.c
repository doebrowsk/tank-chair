#include <stdlib.h>

#include "wagonSteering.h"
#include "packets.h"
#include "vector_math.h"
#include "harlielog.h"
#include <math.h>
#include <vxWorks.h>
#include "differentialSteering.h"

/*
 * TODO Config file
*/
#define HANDLE_LENGTH 1.0
#define REORIENT_DIST 0.25
#define ROTATE_IN_PLACE_DIST 0.1
#define ROTATE_IN_PLACE_HEAD 0.2

int startedReorienting = FALSE;

int _intersectedWithCircle(const Point2d * start, const Point2d * currentLocation, const Vector2d * directionVector, Point2d * const intersectionPoint);
int _rotateInPlace(const Point2d * start, const Point2d * end, const Pose * currentLocation);

// Performs a wagon steering algorithm given the waypoint and last waypoint.
STATUS wagonSteering(const WaypointCommand * lastWaypoint, const WaypointCommand * waypoint, HeadingSpeedCommand * const hscommand, const Pose * pose) 
{
    float x, y;
    float heading = 0.0;
    float speed = 0.0;
    speed = waypoint->max_speed;

    Point2d adjustment;
    Point2d currentLocation;
    currentLocation.x = pose->x;
    currentLocation.y = pose->y;

    float dist = distance(&(waypoint->p), &currentLocation);
    
    if (dist < HANDLE_LENGTH) 
    {
        if ((dist < REORIENT_DIST) || startedReorienting) 
        {
            startedReorienting = TRUE;
            heading = waypoint->heading;
            LOG.VERBOSE("Reorienting to the desired heading");
            speed = 0.0;
        } 
        else if (!startedReorienting)
        {
            adjustment = waypoint->p;
            x = adjustment.x - currentLocation.x;
            y = adjustment.y - currentLocation.y;
            heading = (float) atan2(y, x);
            LOG.VERBOSE("Heading towards the goal point");
            speed = (float) exp(-1.0 * (HANDLE_LENGTH / dist)) * waypoint->max_speed;
        }
        LOG.VERBOSE("Less than handle length. Heading is %f", heading);
        
    } 
    else 
    {
        startedReorienting = FALSE;
        Vector2d d;
        vecFromPoint2ds(&(lastWaypoint->p), &(waypoint->p), &d);
        int intersected = _intersectedWithCircle(&(lastWaypoint->p), &currentLocation, &d, &adjustment);
        if (intersected == TRUE) 
        {
             x = adjustment.x - currentLocation.x;
             y = adjustment.y - currentLocation.y;
             LOG.VERBOSE("Adjustment is adj.x: %f, adj.y: %f, x: %f, y: %f", adjustment.x, adjustment.y, x, y);
             heading = (float) atan2(y, x);
             LOG.VERBOSE("Intersected. Heading is %f", heading);
        }
        else 
        {
            heading = pose->theta;
            LOG.ERR("No Intersection. Heading is %f", heading);
            speed = 0.0;
        }
    }
    /*
     * Check to see if we should override the speed and rotate in place
     */
    if (_rotateInPlace(&(lastWaypoint->p), &(waypoint->p), pose) == TRUE) 
    {
        LOG.VERBOSE("Wagon Steering decided to rotate in place, killing translational speed");
        speed = 0.0;
    }

    LOG.VERBOSE("Wagon Steering: Heading: %f, Speed: %f", heading, speed);
    hscommand->desiredHeading = heading;
    hscommand->desiredSpeed = speed;

    return (OK);
}

// TODO: figure out what this functions does
int _intersectedWithCircle(const Point2d * start, const Point2d * currentLocation, const Vector2d * directionVector, Point2d * const intersectionPoint) {
/*
 * Porting the Java intersection code.
*/
    STATUS err;
    Vector2d d;
    err = normalize(directionVector, &d);
    if (err != OK) 
    {
        LOG.ERR("Error normalizing the direction vector");
    } 
    else 
    {
        LOG.VERBOSE("d.x = %f, d.y = %f", d.p.x, d.p.y);
        Vector2d s, c, v;
        vecFromPoint2d(start, &s);
        LOG.VERBOSE("s.x = %f, s.y = %f", s.p.x, s.p.y);
        vecFromPoint2d(currentLocation, &c);
        LOG.VERBOSE("c.x = %f, c.y = %f", c.p.x, c.p.y);

        sub(&s, &c, &v);
        LOG.VERBOSE("v.x = %f, v.y = %f", v.p.x, v.p.y);

        float temp2 = dot(&v, &d);
        LOG.VERBOSE("temp2 1: %f", temp2);
        temp2 = (float) pow(temp2, 2.0);
        LOG.VERBOSE("temp2 2: %f", temp2);

        float temp3 = dot(&v, &v);
        LOG.VERBOSE("temp3 1: %f", temp3);
        temp3 = temp3 - (float) pow(HANDLE_LENGTH, 2.0);
        LOG.VERBOSE("temp3 2: %f", temp3);
        float discriminant = temp2 - temp3;
        LOG.VERBOSE("discriminant: %f", discriminant);

        if (discriminant < 0.0) 
        {
            return FALSE;
        } 
        else 
        {
            temp2 = -1.0 * dot(&v, &d);
            LOG.VERBOSE("temp2 3: %f", temp2);
            temp3 = (float) sqrt(discriminant);
            LOG.VERBOSE("temp3 3: %f", temp3);
            float t1 = temp2 + temp3;
            LOG.VERBOSE("t1: %f", t1);
            float t2 = temp2 - temp3;
            LOG.VERBOSE("t2: %f", t2);

            float t = max(t1, t2);
            LOG.VERBOSE("t: %f", t);

            if (t < 0.0) 
            {
                if ((t1 < 0.0) && (t2 < 0.0)) 
                {
                    return FALSE;
                } 
                else if ((t1 < 0.0) && (t2 >= 0.0)) 
                {
                    t = t2;
                } 
                else if ((t1 >= 0.0) && (t2 < 0.0)) 
                {
                    t = t1;
                }
            }
            LOG.VERBOSE("t 2: %f", t);
            float xtemp = start->x + t * d.p.x;
            LOG.VERBOSE("xtemp: %f", xtemp);
            float ytemp = start->y + t * d.p.y;
            LOG.VERBOSE("ytemp: %f", ytemp);

            intersectionPoint->x = xtemp;
            intersectionPoint->y = ytemp;

            return TRUE;
        }
    }
    LOG.ERR("Somehow we fell out of the bottom of the _intersectedWithCircle function");
    return FALSE;
}

// Rotates the robot in place and calculates the speed to turn the robot properly
int _rotateInPlace(const Point2d * start, const Point2d * end, const Pose * currentLocation) 
{
/*
    * Implementing line-point distance from mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
*/
    float term1 = (end->x - start->x) * (start->y - currentLocation->y);
    float term2 = (end->y - start->y) * (start->x - currentLocation->x);

    float denominator = (float) sqrt((pow((end->x - start->x), 2.0) - pow((end->y - start->y), 2.0)));

    float numerator = term1 - term2;

    float distance = ((float) fabs(numerator)) / denominator;

    if (distance < ROTATE_IN_PLACE_DIST) 
    {
        float lineHeading = (float) atan2(end->y - start->y, end->x - start->x);
        float min_angle = calculateMinimalSteeringAngle(lineHeading, currentLocation->theta);
        float diff = (float) fabs(min_angle);
        LOG.VERBOSE("Rotate in place calculated minimum angle %f for desired %f given current %f", diff, lineHeading, currentLocation->theta);

        if (diff > ROTATE_IN_PLACE_HEAD) 
        {
            return TRUE;
        }
        else 
        {
            return FALSE;
        }
    } 
    else 
    {
        return FALSE;
    }
}
