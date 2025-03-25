
using FixMath;

namespace Pathfinding.Crowds.MoveStrategy
{
    // 空闲状态下的移动策略: 可以被单位挤开，向目标点聚集
    public class IdleMoveStrategy : AbstractMoveStrategy
    {
        public IdleMoveStrategy(MovableEntity owner)
        {
        }

        public override bool Condition(MovableEntity owner)
        {
            if (!owner.HasEntityState(MovableEntity.eEntityState.Moving))
            {
                return true;
            }
            return false;
        }

        public override void OnStart(MovableEntity owner)
        {
        }

        public override void OnStop(MovableEntity owner)
        {
        }

        public override F64Vec3 OnUpdate(MovableEntity owner, F64 deltaSeconds)
        {
            return FixMath.F64Vec3.Zero;
        }
    }
}
