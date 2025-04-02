
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
        FlockSeparationForce _flockSeparationForce;
        FlockAlignmentForce _flockAlignmentForce;
        FlockCohesionForce _flockCohesionForce;

        FixMath.F64 _predictionAvoidIdleNeighborTime = FixMath.F64.FromDouble(0.05);

        public FollowPathMoveStrategy(MovableEntity owner)
        {
            var template = owner.Template as TMovableEntityTemplate;

            _followPathForce = new FollowPathForce();
            _followPathForce.FollowPathAheadTime = template.FollowPathAheadTime;
            _followPathForce.Weight = template.FollowPathWeight;

            _avoidIdleNeighborForce = new AvoidIdleNeighborForce();
            _avoidIdleNeighborForce.AvoidNeighborAheadTime = template.AvoidNeighborAheadTime;
            _avoidIdleNeighborForce.CheckResetAvoidInfoDistance = template.Radius * 6;
            _avoidIdleNeighborForce.Weight = template.AvoidNeighborWeight;

            _predictionAvoidIdleNeighborTime = template.PredictionAvoidIdleNeighborTime;

            _avoidObstacleForce = new AvoidObstacleForce();
            _avoidObstacleForce.AvoidObstacleAheadTime = template.AvoidObstacleAheadTime;
            _avoidObstacleForce.CheckResetAvoidInfoDistance = template.Radius * 6;
            _avoidObstacleForce.Weight = template.AvoidObstacleWeight;

            _forwadMoveForce = new ForwardMoveForce();
            _forwadMoveForce.Weight = template.ForwardMoveWeight;

            _flockSeparationForce = new FlockSeparationForce();
            _flockSeparationForce.SeparationRadius = template.SeparationRadius;
            _flockSeparationForce.SeparationAngle = template.SeparationAngle;
            _flockSeparationForce.Weight = template.SeparationWeight;

            _flockAlignmentForce = new FlockAlignmentForce();
            _flockAlignmentForce.AlignmentRadius = template.AlignmentRadius;
            _flockAlignmentForce.AlignmentAngle = template.AlignmentAngle;
            _flockAlignmentForce.Weight = template.AlignmentWeight;

            _flockCohesionForce = new FlockCohesionForce();
            _flockCohesionForce.CohesionRadius = template.CohesionRadius;
            _flockCohesionForce.CohesionAngle = template.CohesionAngle;
            _flockCohesionForce.Weight = template.CohesionWeight;

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
            _flockSeparationForce.Reset();
            _flockAlignmentForce.Reset();
            _flockCohesionForce.Reset();
        }

        public override void OnStop(MovableEntity owner)
        {
        }

        public override F64Vec3 OnUpdate(MovableEntity owner, F64 deltaSeconds)
        {
            var followPathForce = _followPathForce.GetSteeringForce(owner);

            _avoidIdleNeighborForce.UpdateAvoidNeighborInfo(owner, _avoidObstacleForce.AvoidObstacleInfo);
            var avoidIdleForce = _avoidIdleNeighborForce.GetSteeringForce(owner);

            _avoidObstacleForce.UpdateAvoidObstacleInfo(owner, _avoidIdleNeighborForce.AvoidNeighborInfo);
            var avoidObstacleForce = _avoidObstacleForce.GetSteeringForce(owner);

            var forwardMoveForce = FixMath.F64Vec3.Zero;
            var flockSeparationForce = FixMath.F64Vec3.Zero;
            var flockAlignmentForce = FixMath.F64Vec3.Zero;
            var flockCohesionForce = FixMath.F64Vec3.Zero;
            if (avoidIdleForce != FixMath.F64Vec3.Zero ||
                avoidObstacleForce != FixMath.F64Vec3.Zero)
            {
                // 如果有避让单位，给一个前向驱动的力
                forwardMoveForce = _forwadMoveForce.GetSteeringForce(owner);
                // 不应用路径跟随力(只在有避让Idle单位时候)
                if (avoidIdleForce != FixMath.F64Vec3.Zero)
                    followPathForce = FixMath.F64Vec3.Zero;
            }
            else 
            {
                // 没有避让，则施加群体行为驱动力
                flockSeparationForce = _flockSeparationForce.GetSteeringForce(owner);
                flockAlignmentForce = _flockAlignmentForce.GetSteeringForce(owner);
                flockCohesionForce = _flockCohesionForce.GetSteeringForce(owner);

                // 检查下一帧是否会避让邻近单位
                if (_predictionAvoidIdleNeighborTime > 0)
                {
                    var totalForce = followPathForce + avoidIdleForce + avoidObstacleForce + forwardMoveForce
                        + flockSeparationForce + flockAlignmentForce + flockCohesionForce;
                    if (totalForce != FixMath.F64Vec3.Zero)
                    {
                        var predictionAvoidIdleForce = PredictionAvoidIdleNeighborsFoce(owner, totalForce, _predictionAvoidIdleNeighborTime, _avoidIdleNeighborForce.AvoidNeighborAheadTime);
                        if (predictionAvoidIdleForce != FixMath.F64Vec3.Zero)
                        {
                            // 应用前驱力
                            forwardMoveForce = _forwadMoveForce.GetSteeringForce(owner);

                            // 不应用路径跟随力和聚集力
                            followPathForce = FixMath.F64Vec3.Zero;
                            flockCohesionForce = FixMath.F64Vec3.Zero;
                        }
                    }
                }
            }
            return followPathForce + avoidIdleForce + avoidObstacleForce + forwardMoveForce 
                + flockSeparationForce + flockAlignmentForce + flockCohesionForce;
        }

        public override void DrawGizmos(MovableEntity owner)
        {
            _followPathForce.DrawGizmos(owner);
            _avoidIdleNeighborForce.DrawGizmos(owner);
            _avoidObstacleForce.DrawGizmos(owner);
            _forwadMoveForce.DrawGizmos(owner);
            _flockSeparationForce.DrawGizmos(owner);
            _flockAlignmentForce.DrawGizmos(owner);
            _flockCohesionForce.DrawGizmos(owner);
        }
    }
}
