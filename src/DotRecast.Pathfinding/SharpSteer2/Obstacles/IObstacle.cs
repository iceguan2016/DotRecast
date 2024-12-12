// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using SharpSteer2.Helpers;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;

namespace SharpSteer2.Obstacles
{
    // PathIntersection object: used internally to analyze and store
    // information about intersections of vehicle paths and obstacles.
    public struct PathIntersection
    {
        public static PathIntersection DEFAULT = new PathIntersection();

        public bool intersect; // was an intersection found?
        public float distance; // how far was intersection point from vehicle?
        public Vector3 surfacePoint; // position of intersection
        public Vector3 surfaceNormal; // unit normal at point of intersection
        public Vector3 steerHint; // where to steer away from intersection
        public bool vehicleOutside; // is the vehicle outside the obstacle?
        public IObstacle obstacle; // obstacle the path intersects

        // determine steering based on path intersection tests
        public Vector3 steerToAvoidIfNeeded(IVehicle vehicle,
                                       float minTimeToCollision,
                                       Vector3? referencePoint = null)
        {
            // if nearby intersection found, steer away from it, otherwise no steering
            float minDistanceToCollision = minTimeToCollision * vehicle.Speed;
            if (intersect && (distance < minDistanceToCollision))
            {
                // compute avoidance steering force: take the component of
                // steerHint which is lateral (perpendicular to vehicle's
                // forward direction), set its length to vehicle's maxForce
                if (referencePoint != null)
                {
                    // Select the direction that is closer to the reference point
                    Vector3 d = referencePoint.Value - surfacePoint;
                    Vector3 hintDir = Vector3.Dot(d, steerHint) >= 0 ? steerHint : (2 * Vector3.Dot(steerHint, surfaceNormal) * surfaceNormal - steerHint);
                    Vector3 lateral = Vector3Helpers.PerpendicularComponent(hintDir, vehicle.Forward);
                    return Vector3.Normalize(lateral) * vehicle.MaxForce;
                }
                else
                {
                    Vector3 lateral = Vector3Helpers.PerpendicularComponent(steerHint, vehicle.Forward);
                    return Vector3.Normalize(lateral) * vehicle.MaxForce;
                }
            }
            else
            {
                return Vector3.Zero;
            }
        }
    };

    // seenFrom (eversion): does this obstacle contrain vehicle to stay
    // inside it or outside it (or both)?  "Inside" describes a clear space
    // within a solid (for example, the interior of a room inside its
    // walls). "Ouitside" describes a solid chunk in the midst of clear
    // space.
    public enum seenFromState { outside, inside, both };

    // ----------------------------------------------------------------------------
    // AbstractObstacle: a pure virtual base class for an abstract shape in
    // space, to be used with obstacle avoidance.  (Oops, its not "pure" since
    // I added a concrete method to PathIntersection 11-04-04 -cwr).

    public enum ObstacleType
    {
        None = 0,
        Sphere,
        Box,
        Plane,
        Rectangle
    };

    /// <summary>
    /// Obstacle: a pure virtual base class for an abstract shape in space, to be
    /// used with obstacle avoidance.
    /// </summary>
    public interface IObstacle
	{
        Vector3 steerToAvoid(BaseVehicle v, float minTimeToCollision, Vector3? referencePoint = null);

        // find first intersection of a vehicle's path with this obstacle
        // (this must be specialized for each new obstacle shape class)
        void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi);

        // virtual function for drawing -- normally does nothing, can be
        // specialized by derived types to provide graphics for obstacles
        void draw(bool filled, Vector4 color, Vector3 viewpoint);

        seenFromState seenFrom();
        void setSeenFrom(seenFromState s);

        ObstacleType getObstacleType();
    }
}
