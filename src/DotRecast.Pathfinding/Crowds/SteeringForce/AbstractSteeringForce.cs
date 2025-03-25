
namespace Pathfinding.Crowds.SteeringForce
{
    public abstract class AbstractSteeringForce : ISteeringForce
    {
        public FixMath.F64 Weight { get; set; }

        public abstract FixMath.F64Vec3 GetSteeringForce(MovableEntity owner);
    }
}
