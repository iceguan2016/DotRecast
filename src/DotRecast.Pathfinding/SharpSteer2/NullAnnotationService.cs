
using SharpSteer2.Obstacles;
using System.Numerics;

namespace SharpSteer2
{
    class NullAnnotationService
        :IAnnotationService
    {
        public bool IsEnabled
        {
            get
            {
                return false;
            }
            set
            {
            }
        }

        public void Line(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec3 color, FixMath.F64 opacity /*= 1*/)
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

        public void SolidPlane(FixMath.F64Vec3 point, FixMath.F64Vec3 normal, FixMath.F64Vec2 size, FixMath.F64Vec3 color, FixMath.F64 opacity)
        { 
        }

        public void AvoidObstacle(IVehicle self, FixMath.F64 minDistanceToCollision)
        {

        }

        public void AvoidObstacle(IVehicle self, FixMath.F64 minDistanceToCollision, PathIntersection nearest)
        {
        }

        public void PathFollowing(IVehicle self, FixMath.F64Vec3 future, FixMath.F64Vec3 onPath, FixMath.F64Vec3 target, FixMath.F64 outside)
        {

        }

        public void AvoidCloseNeighbor(IVehicle self, IVehicle other, FixMath.F64 additionalDistance)
        {

        }

        public void AvoidNeighbor(IVehicle self, IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture)
        {

        }

        public void AvoidNeighbor2(IVehicle self, IVehicle threat, PathIntersection intersection)
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
