
using FixMath;
using SharpSteer2;

namespace Pathfinding.Crowds.SteeringForce
{
    public class AvoidObstacleForce : AbstractSteeringForce
    {
        // Avoid obstacle look-ahead time
        public FixMath.F64 AvoidObstacleAheadTime = FixMath.F64.FromDouble(1.0f);
        public FixMath.F64 CheckResetAvoidInfoDistance = FixMath.F64.One;

        private IVehicle.FAvoidObstacleInfo _avoidObstacleInfo = new IVehicle.FAvoidObstacleInfo();
        public IVehicle.FAvoidObstacleInfo AvoidObstacleInfo { get { return _avoidObstacleInfo; } }

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var boundaryObstacles = owner.BoundaryObstacles;
            var obstacleAvoidance = FixMath.F64Vec3.Zero;
            {
                if (boundaryObstacles.Count > 0)
                {
                    obstacleAvoidance = owner.SteerToAvoidObstacles(AvoidObstacleAheadTime, boundaryObstacles, ref _avoidObstacleInfo) * Weight;
                }
            }
            return obstacleAvoidance;
        }

        public override void Reset()
        {
            _avoidObstacleInfo.Reset();
        }

        public void UpdateAvoidObstacleInfo(MovableEntity owner, IVehicle.FAvoidNeighborInfo referenceAoidNeighborInfo)
        {
            var entityManager = owner.EntityManager;

            if (null == _avoidObstacleInfo.Obstacle)
            {
                _avoidObstacleInfo.Reset();
            }
            else
            {
                var distance = _avoidObstacleInfo.Obstacle.pointToObstacleDistance(owner.Position);
                if (distance >= CheckResetAvoidInfoDistance)
                {
                    _avoidObstacleInfo.Reset();
                }
            }

            // 根据传入的FAvoidNeighborInfo信息更新避让初始信息，防止从避让Entity过渡到Obstacle方向不一致导致单位原地打转
            if (!_avoidObstacleInfo.IsValid &&
                referenceAoidNeighborInfo.IsValid && 
                referenceAoidNeighborInfo.FrameNo > _avoidObstacleInfo.FrameNo)
            {
                _avoidObstacleInfo.SetAvoidInfo(entityManager.FrameNo, null, referenceAoidNeighborInfo.Side);
            }
        }
    }
}
