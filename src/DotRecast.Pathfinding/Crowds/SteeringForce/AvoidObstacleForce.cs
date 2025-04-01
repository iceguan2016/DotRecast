
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

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            updateAvoidObstacleInfo(owner);

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

        void updateAvoidObstacleInfo(MovableEntity owner)
        {
            var entityManager = owner.EntityManager;

            if (null == _avoidObstacleInfo.Obstacle)
            {
                _avoidObstacleInfo.Reset();
                return;
            }

            var distance = _avoidObstacleInfo.Obstacle.pointToObstacleDistance(owner.Position);
            if (distance >= CheckResetAvoidInfoDistance)
            {
                _avoidObstacleInfo.Reset();
                return;
            }
        }
    }
}
