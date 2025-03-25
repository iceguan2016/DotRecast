
using FixMath;
using Pathfinding.Crowds.SteeringForce;

namespace Pathfinding.Crowds.MoveStrategy
{
    // 跟随导航路径移动策略: 避让障碍物，静止/移动中单位，群体行为
    public class FollowPathMoveStrategy : AbstractMoveStrategy
    {
        FollowPathForce _followPathForce;
        AvoidIdleNeighborForce _avoidIdleNeighborForce;
        AvoidObstacleForce _avoidObstacleForce;
        ForwardMoveForce _forwadMoveForce;
        GroupFlockForce _groupFlockForce;

        public FollowPathMoveStrategy(MovableEntity owner)
        {
            var template = owner.Template as TMovableEntityTemplate;

            _followPathForce = new FollowPathForce();
            _followPathForce.FollowPathAheadTime = template.FollowPathAheadTime;
            _followPathForce.Weight = template.FollowPathWeight;

            _avoidIdleNeighborForce = new AvoidIdleNeighborForce();
            _avoidIdleNeighborForce.AvoidNeighborAheadTime = template.AvoidNeighborAheadTime;
            _avoidIdleNeighborForce.Weight = template.AvoidNeighborWeight;

            _avoidObstacleForce = new AvoidObstacleForce();
            _avoidObstacleForce.AvoidObstacleAheadTime = template.AvoidObstacleAheadTime;
            _avoidObstacleForce.Weight = template.AvoidObstacleWeight;

            _forwadMoveForce = new ForwardMoveForce();
            _forwadMoveForce.Weight = template.ForwardMoveWeight;

            _groupFlockForce = new GroupFlockForce();
            _groupFlockForce.SeparationRadius = template.SeparationRadius;
            _groupFlockForce.SeparationAngle = template.SeparationAngle;
            _groupFlockForce.SeparationWeight = template.SeparationWeight;

            _groupFlockForce.AlignmentRadius = template.AlignmentRadius;
            _groupFlockForce.AlignmentAngle = template.AlignmentAngle;
            _groupFlockForce.AlignmentWeight = template.AlignmentWeight;

            _groupFlockForce.CohesionRadius = template.CohesionRadius;
            _groupFlockForce.CohesionAngle = template.CohesionAngle;
            _groupFlockForce.CohesionWeight = template.CohesionWeight;

        }

        public override bool Condition(MovableEntity owner)
        {
            if (owner.HasEntityState(MovableEntity.eEntityState.Moving))
            {
                return true;
            }
            return false;
        }

        public override void OnStart(MovableEntity owner)
        {
            _followPathForce.Reset();
            _avoidIdleNeighborForce.Reset();
            _avoidObstacleForce.Reset();
            _forwadMoveForce.Reset();
            _groupFlockForce.Reset();
        }

        public override void OnStop(MovableEntity owner)
        {
        }

        public override F64Vec3 OnUpdate(MovableEntity owner, F64 deltaSeconds)
        {
            var followPathForce = _followPathForce.GetSteeringForce(owner);
            var avoidIdleForce = _avoidIdleNeighborForce.GetSteeringForce(owner);
            var avoidObstacleForce = _avoidObstacleForce.GetSteeringForce(owner);

            var forwardMoveForce = FixMath.F64Vec3.Zero;
            var flockMoveForce = FixMath.F64Vec3.Zero;
            if (avoidIdleForce != FixMath.F64Vec3.Zero ||
                avoidObstacleForce != FixMath.F64Vec3.Zero)
            {
                // 如果有避让单位，给一个前向驱动的力
                forwardMoveForce = _forwadMoveForce.GetSteeringForce(owner);
            }
            else
            {
                // 没有避让，则施加群体行为驱动力
                flockMoveForce = _groupFlockForce.GetSteeringForce(owner);
            }
            return followPathForce + avoidIdleForce + avoidObstacleForce + forwardMoveForce + flockMoveForce;
        }

        public override void DrawGizmos(MovableEntity owner)
        {
            _followPathForce.DrawGizmos(owner);
            _avoidIdleNeighborForce.DrawGizmos(owner);
            _avoidObstacleForce.DrawGizmos(owner);
            _forwadMoveForce.DrawGizmos(owner);
            _groupFlockForce.DrawGizmos(owner);
        }
    }
}
