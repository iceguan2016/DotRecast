
using SharpSteer2;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using Volatile;
using static System.Net.Mime.MediaTypeNames;

namespace Pathfinding.Crowds.SteeringForce
{
    public abstract class AbstractSteeringForce : ISteeringForce, IVoltPoolable<AbstractSteeringForce>
    {
        // IVoltPoolable interface
        public IVoltPool<AbstractSteeringForce> Pool { get; set; }
        public abstract void Reset();
        // End

        public FixMath.F64 Weight { get; set; }

        public static bool CheckGroupShouldAvoid(MovableEntity owner, MovableEntity other)
        {
            return owner.GroupsToAvoid > 0 && (owner.GroupsToAvoid & other.GroupMask) > 0;
        }
        public static PathIntersection CheckHitObstacle(MovableEntity owner, FixMath.F64Vec3 dir)
        {
            var q = FixMath.F64Quat.LookRotation(dir, FixMath.F64Vec3.Up);
            var vehicle = new SimpleVehicle();
            vehicle.Position = owner.Position;
            vehicle.Radius = owner.Radius;
            vehicle.Forward = q * FixMath.F64Vec3.AxisZ;
            vehicle.Up = q * FixMath.F64Vec3.AxisY;
            vehicle.Side = q * FixMath.F64Vec3.AxisX;
            Obstacle.firstPathIntersectionWithObstacleGroup(vehicle, owner.BoundaryObstacles, out var nearest, out var next);
            return nearest;
        }

        public abstract FixMath.F64Vec3 GetSteeringForce(MovableEntity owner);

        public virtual void DrawGizmos(MovableEntity owner) { }
    }
}
