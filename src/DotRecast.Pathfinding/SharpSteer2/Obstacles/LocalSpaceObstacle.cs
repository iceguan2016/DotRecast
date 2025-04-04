using System;
using System.Numerics;
using FixMath;
using SharpSteer2.Helpers;

namespace SharpSteer2.Obstacles
{
    // LocalSpaceObstacle: a mixture of LocalSpace and Obstacle methods
    public abstract class LocalSpaceObstacle : LocalSpace, IObstacle
    {
        public abstract void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi);
        public abstract F64 pointToObstacleDistance(F64Vec3 p);

        // compute steering for a vehicle to avoid this obstacle, if needed 
        public FixMath.F64Vec3 steerToAvoid(BaseVehicle vehicle, FixMath.F64 minTimeToCollision, ref IVehicle.FAvoidObstacleInfo info)
        {
            // find nearest intersection with this obstacle along vehicle's path
            PathIntersection pi = PathIntersection.DEFAULT;
            findIntersectionWithVehiclePath(vehicle, ref pi);

            // return steering for vehicle to avoid intersection, or zero if non found
            return pi.steerToAvoidIfNeeded(vehicle, minTimeToCollision, ref info);
        }

        public abstract void draw(IAnnotationService annotation, bool filled, FixMath.F64Vec3 color, FixMath.F64Vec3 viewpoint);

        public seenFromState seenFrom() { return _seenFrom; }
        public void setSeenFrom(seenFromState s) { _seenFrom = s; }

        public abstract ObstacleType getObstacleType();

        private seenFromState _seenFrom = seenFromState.outside;
    }
}
