
using FixMath;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class AvoidMoveNeighborForce : AbstractSteeringForce
    { 
        // 方向角度阈值
        public FixMath.F64 DirctionAngleThreshold = FixMath.F64.DegToRad(FixMath.F64.FromFloat(45.0f));
        FixMath.F64 DirctionAngleThresholdCos;

        public AvoidMoveNeighborForce()
        {
            DirctionAngleThresholdCos = FixMath.F64.CosFast(DirctionAngleThreshold);
        }

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            return FixMath.F64Vec3.Zero;
        }

        public override void Reset()
        {
        }

        bool NeedAvoidNeighbor(MovableEntity owner, MovableEntity neighbor)
        {
            // 移动方向同向的不避让，移动方向反向的需避让
            if (!neighbor.HasEntityState(MovableEntity.eEntityState.Moving))
            {
                return true;
            }

            var dir0 = owner.Velocity.GetSafeNormal();
            var dir1 = neighbor.Velocity.GetSafeNormal();
            var dot = dir0.Dot(dir1);
            if (dot < DirctionAngleThresholdCos)
            {
                return true;
            }
            return false;
        }
    }
}
