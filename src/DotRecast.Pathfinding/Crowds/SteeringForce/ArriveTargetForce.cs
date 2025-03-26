
using FixMath;

namespace Pathfinding.Crowds.SteeringForce
{
    public class ArriveTargetForce : AbstractSteeringForce
    {
        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var steering = FixMath.F64Vec3.Zero;
            if (null != owner.TargetLocation)
            {
                steering = owner.SteerForSeek(owner.TargetLocation.Value) * Weight;
            }
            return steering;
        }

        public override void Reset()
        {
        }
    }
}
