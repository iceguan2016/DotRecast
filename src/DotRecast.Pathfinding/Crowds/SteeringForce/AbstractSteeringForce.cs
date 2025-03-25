
using Volatile;

namespace Pathfinding.Crowds.SteeringForce
{
    public abstract class AbstractSteeringForce : ISteeringForce, IVoltPoolable<AbstractSteeringForce>
    {
        // IVoltPoolable interface
        public IVoltPool<AbstractSteeringForce> Pool { get; set; }
        public abstract void Reset();
        // End

        public FixMath.F64 Weight { get; set; }

        public abstract FixMath.F64Vec3 GetSteeringForce(MovableEntity owner);

        public virtual void DrawGizmos(MovableEntity owner) { }
    }
}
