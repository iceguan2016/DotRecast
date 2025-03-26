
using FixMath;
using Pathfinding.Crowds.SteeringForce;

namespace Pathfinding.Crowds.MoveStrategy
{
    // 空闲状态下的移动策略: 可以被单位挤开，向目标点聚集
    public class IdleMoveStrategy : AbstractMoveStrategy
    {
        ArriveTargetForce _holdTargetForce;
        FlockSeparationForce _separationForce;
        FlockCohesionForce _cohesionForce;

        FixMath.F64 _stopMoveRadius;

        public IdleMoveStrategy(MovableEntity owner)
        {
            var template = owner.Template as TMovableEntityTemplate;

            _holdTargetForce = new ArriveTargetForce();
            _holdTargetForce.Weight = FixMath.F64.FromDouble(1.0f);

            _separationForce = new FlockSeparationForce();
            _separationForce.Weight = FixMath.F64.FromDouble(1.0f);

            _cohesionForce = new FlockCohesionForce();
            _cohesionForce.Weight = FixMath.F64.FromDouble(1.0f);

            _stopMoveRadius = template.StopMoveRadius;
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
            // 超出停止距离了才施加力
            if (null != owner.TargetLocation)
            {
                var distance = FixMath.F64Vec3.DistanceFast(owner.Position, owner.TargetLocation.Value);
                if (distance > _stopMoveRadius)
                {
                    var holdTargetForce = _holdTargetForce.GetSteeringForce(owner);
                    var separationForce = _separationForce.GetSteeringForce(owner);
                    var cohesionForce = _cohesionForce.GetSteeringForce(owner);
                    return holdTargetForce + separationForce + cohesionForce;
                }
            }

            return FixMath.F64Vec3.Zero;
        }
    }
}
