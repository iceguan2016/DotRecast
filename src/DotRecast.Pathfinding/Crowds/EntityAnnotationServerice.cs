using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using SharpSteer2;
using SharpSteer2.Obstacles;

namespace DotRecast.Pathfinding.Crowds
{
    public class EntityAnnotationServerice : IAnnotationService
    {
        Game.Utils.DrawInterface _draw = null;

        public EntityAnnotationServerice(Game.Utils.DrawInterface inDraw)
        {
            _draw = inDraw;
        }

        public bool IsEnabled { get; set; }

        public void Line(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec3 color, FixMath.F64 opacity)
        {
        }

        public void CircleXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments)
        {
        }

        public void DiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments)
        {
        }

        public void Circle3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments)
        {
        }

        public void Disk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments)
        {
        }

        public void CircleOrDiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled)
        {
        }

        public void CircleOrDisk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments, bool filled)
        {
        }

        public void CircleOrDisk(FixMath.F64 radius, FixMath.F64Vec3 axis, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled, bool in3D)
        {
        }

        public void AvoidObstacle(FixMath.F64 minDistanceToCollision)
        {
        }

        public void AvoidObstacle(FixMath.F64 minDistanceToCollision, SharpSteer2.Obstacles.PathIntersection nearest)
        {
        }

        public void PathFollowing(FixMath.F64Vec3 future, FixMath.F64Vec3 onPath, FixMath.F64Vec3 target, FixMath.F64 outside)
        {
        }

        public void AvoidCloseNeighbor(IVehicle other, FixMath.F64 additionalDistance)
        {
        }

        public void AvoidNeighbor(IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture)
        {
        }

        public void AvoidNeighbor2(IVehicle threat, PathIntersection intersection)
        {
        }

        public void VelocityAcceleration(IVehicle vehicle)
        {
        }

        public void VelocityAcceleration(IVehicle vehicle, FixMath.F64 maxLength)
        {
        }

        public void VelocityAcceleration(IVehicle vehicle, FixMath.F64 maxLengthAcceleration, FixMath.F64 maxLengthVelocity)
        {
        }
    }
}
