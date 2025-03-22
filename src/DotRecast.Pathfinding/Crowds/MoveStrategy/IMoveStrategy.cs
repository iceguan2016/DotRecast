
namespace Pathfinding.Crowds.AvoidStrategy
{
    // 移动策略，单位在不同阶段有不同的移动规则，比如Idle状态可以被别的单位推挤开，推挤完毕后回到之前位置，
    // Moving状态需要避让对向单位，障碍物等等
    public interface IMoveStrategy
    {
        // 检查策略是否满足条件
        bool Condition(MovableEntity owner);
        //
        void OnStart(MovableEntity owner);
        // 返回最终的SteeringForce
        FixMath.F64Vec3 OnUpdate(MovableEntity owner, FixMath.F64 deltaSeconds);
        //
        void OnStop(MovableEntity owner);

        // 通知事件
        void OnOverlapBegin(MovableEntity owner, MovableEntity other);
        void OnOverlapEnd(MovableEntity owner, MovableEntity other);
    }
}
