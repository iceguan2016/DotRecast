
using FixMath;

namespace Pathfinding.Crowds.MoveStrategy
{
    // 攻击状态下的移动策略: 不可被单位推挤开，其他单位需要绕行
    public class AttackMoveStrategy : AbstractMoveStrategy
    {
        public AttackMoveStrategy(MovableEntity owner)
        { 
        }

        public override bool Condition(MovableEntity owner)
        {
            if (owner.HasEntityState(MovableEntity.eEntityState.Attack))
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
