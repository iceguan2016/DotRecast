using System;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;

namespace Pathfinding.Crowds
{
    public class EntityAnnotationServerice : IAnnotationService
    {
        DrawInterface _draw = null;

        public EntityAnnotationServerice(DrawInterface inDraw)
        {
            _draw = inDraw;
        }

        public bool IsEnabled { get; set; }

        public void Line(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec3 color, FixMath.F64 opacity)
        {
            if (_draw != null) 
            {
                _draw.DrawLine(startPoint.Cast(), endPoint.Cast(), color.Cast(opacity));
            }
        }

        public void Arrow(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec2 arrowSize, FixMath.F64 lineWidth, FixMath.F64Vec3 color, FixMath.F64 opacity)
        {
            if (_draw != null) 
            {
                _draw.DrawArrow(startPoint.Cast(), endPoint.Cast(), arrowSize.Cast(), lineWidth.Float, color.Cast(opacity));
            }
        }

        public void CircleXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments)
        {
            if (_draw != null) 
            {
                _draw.DrawCircle(center.Cast(), radius.Float, color.Cast(FixMath.F64.One));
            }
        }

        public void DiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void Circle3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void Disk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void CircleOrDiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void CircleOrDisk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments, bool filled)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void CircleOrDisk(FixMath.F64 radius, FixMath.F64Vec3 axis, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled, bool in3D)
        {
            CircleXZ(radius, center, color, segments);
        }

        public void SolidPlane(FixMath.F64Vec3 point, FixMath.F64Vec3 normal, FixMath.F64Vec2 size, FixMath.F64Vec3 color, FixMath.F64 opacity)
        {
            if (_draw != null)
            {
                _draw.DrawSolidPlane(point.Cast(), normal.Cast(), size.Cast(), color.Cast(opacity));
            }
        }

        public void SolidCube(FixMath.F64Vec3 center, FixMath.F64Vec3 size, FixMath.F64Vec3 color, FixMath.F64 opacity)
        {
            if (_draw != null)
            {
                _draw.DrawSolidCube(center.Cast(), UnityEngine.Quaternion.identity, size.Cast(), color.Cast(opacity));
            }
        }

        public void AvoidObstacle(IVehicle vehicle, FixMath.F64 minDistanceToCollision)
        {
            vehicle.AnnotationAvoidObstacle(minDistanceToCollision);
        }

        public void AvoidObstacle(IVehicle vehicle, FixMath.F64 minDistanceToCollision, SharpSteer2.Obstacles.PathIntersection nearest)
        {
            vehicle.AnnotationAvoidObstacle(minDistanceToCollision, nearest);
        }

        public void PathFollowing(IVehicle vehicle, FixMath.F64Vec3 future, FixMath.F64Vec3 onPath, FixMath.F64Vec3 target, FixMath.F64 outside)
        {
            var position = vehicle.Position;
            // draw line from our position to our predicted future position
            Line(position, future, Colors.Yellow, FixMath.F64.One);

            // draw line from our position to our steering target on the path
            Line(position, target, Colors.YellowOrange, FixMath.F64.One);

            // draw a two-toned line between the future test point and its
            // projection onto the path, the change from dark to light color
            // indicates the boundary of the tube.
            var boundaryOffset = (onPath - future).Normalize() * outside;
            var onPathBoundary = future + boundaryOffset;
            Line(onPath, onPathBoundary, Colors.DarkOrange, FixMath.F64.One);
            Line(onPathBoundary, future, Colors.LightOrange, FixMath.F64.One);
        }

        public void AvoidCloseNeighbor(IVehicle vehicle, IVehicle other, FixMath.F64 additionalDistance)
        {
            vehicle.AnnotationAvoidCloseNeighbor(other, additionalDistance);
        }

        public void AvoidCloseNeighbor(IVehicle vehicle, IVehicle threat, FixMath.F64Vec3 avoidDirection, IVehicle.FAvoidNeighborInfo info)
        {
            vehicle.AnnotationAvoidCloseNeighbor(threat, avoidDirection, info);
        }

        public void AvoidNeighbor(IVehicle vehicle, IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture)
        {
            vehicle.AnnotationAvoidNeighbor(threat, steer, ourFuture, threatFuture);
        }

        public void AvoidNeighbor(IVehicle vehicle, IVehicle threat, PathIntersection intersection)
        {
            vehicle.AnnotationAvoidNeighbor(threat, intersection);
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
