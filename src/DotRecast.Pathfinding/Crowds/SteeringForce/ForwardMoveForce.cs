
using FixMath;

namespace Pathfinding.Crowds.SteeringForce
{
    // 前向移动驱动力
    public class ForwardMoveForce : AbstractSteeringForce
    {
        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var forwardMoveForce = FixMath.F64Vec3.Zero;
            {
                forwardMoveForce = owner.Forward * owner.MaxForce * Weight;
            }
            return forwardMoveForce;
        }

        public override void Reset()
        {
        }
    }
}
