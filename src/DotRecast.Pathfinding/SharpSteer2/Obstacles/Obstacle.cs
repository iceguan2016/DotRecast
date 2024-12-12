using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using System.Collections.Generic;
using System.Numerics;

namespace SharpSteer2.Obstacles
{
    // ----------------------------------------------------------------------------
    // Obstacle is a utility base class providing some shared functionality


    public abstract class Obstacle : IObstacle
    {
        public abstract void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi);

        // compute steering for a vehicle to avoid this obstacle, if needed 
        public Vector3 steerToAvoid(BaseVehicle vehicle, float minTimeToCollision, Vector3? referencePoint = null)
        {
            // find nearest intersection with this obstacle along vehicle's path
            PathIntersection pi = PathIntersection.DEFAULT;
            findIntersectionWithVehiclePath(vehicle, ref pi);

            // return steering for vehicle to avoid intersection, or zero if non found
            return pi.steerToAvoidIfNeeded(vehicle, minTimeToCollision, referencePoint);
        }

        // static method to apply steerToAvoid to nearest obstacle in an
        // ObstacleGroup
        public static Vector3 steerToAvoidObstacles(BaseVehicle vehicle,
                                               float minTimeToCollision,
                                               IEnumerable<IObstacle> obstacles,
                                               out PathIntersection outNearest,
                                               Vector3? referencePoint = null)
        {
            // test all obstacles in group for an intersection with the vehicle's
            // future path, select the one whose point of intersection is nearest
            firstPathIntersectionWithObstacleGroup(vehicle, obstacles, out var nearest, out var next);

            // if nearby intersection found, steer away from it, otherwise no steering
            outNearest = nearest;
            return nearest.steerToAvoidIfNeeded(vehicle, minTimeToCollision, referencePoint);
        }

        // static method to find first vehicle path intersection in an
        // ObstacleGroup
        public static void firstPathIntersectionWithObstacleGroup(BaseVehicle vehicle,
                                                    IEnumerable<IObstacle> obstacles,
                                                    out PathIntersection nearest,
                                                    out PathIntersection next)
        {
            nearest = PathIntersection.DEFAULT;
            next = PathIntersection.DEFAULT;

            // test all obstacles in group for an intersection with the vehicle's
            // future path, select the one whose point of intersection is nearest
            next.intersect = false;
            nearest.intersect = false;
            foreach (var o in obstacles)
            {
                // find nearest point (if any) where vehicle path intersects obstacle
                // o, storing the results in PathIntersection object "next"
                o.findIntersectionWithVehiclePath(vehicle, ref next);

                // if this is the first intersection found, or it is the nearest found
                // so far, store it in PathIntersection object "nearest"
                bool firstFound = !nearest.intersect;
                bool nearestFound = (next.intersect &&
                                           (next.distance < nearest.distance));
                if (firstFound || nearestFound) nearest = next;
            }
        }

        // default do-nothing draw function (derived class can overload this)
        public abstract void draw(bool filled, Vector4 color, Vector3 viewpoint);

        public seenFromState seenFrom() { return _seenFrom; }
        public void setSeenFrom(seenFromState s) { _seenFrom = s; }

        public abstract ObstacleType getObstacleType();


        private seenFromState _seenFrom = seenFromState.outside;
    };
}
