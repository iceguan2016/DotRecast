

using System.Collections.Generic;
using FixMath;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class AvoidIdleNeighborForce : AbstractSteeringForce
    {
        // Avoid neighbor look-ahead time
        public FixMath.F64 AvoidNeighborAheadTime = FixMath.F64.FromDouble(1.0f);
        public FixMath.F64 CheckHitObstacleTime = FixMath.F64.FromDouble(1.5f);
        public FixMath.F64 CheckResetAvoidInfoDistance = FixMath.F64.One;

        private AvoidanceQuerySystem _avoidQuerySystem = new AvoidanceQuerySystem();
        private List<UniqueId> _avoidNeghborIDs = new List<UniqueId>();

        private IVehicle.FAvoidNeighborInfo _avoidNeighborInfo = new IVehicle.FAvoidNeighborInfo();
        public IVehicle.FAvoidNeighborInfo AvoidNeighborInfo { get { return _avoidNeighborInfo; } }

        bool NeedAvoidNeighbor(MovableEntity owner, MovableEntity neighbor)
        {
            if (!neighbor.HasEntityState(MovableEntity.eEntityState.Moving))
            {
                return true;
            }

            return false;
        }

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var entityManager = owner.EntityManager;
            var neighbors = owner.Neighbors;
            var collisionAvoidance = FixMath.F64Vec3.Zero;
            {
                _avoidNeghborIDs.Clear();
                _avoidQuerySystem.Init(owner.ID, owner.Position.Cast2D(), owner.Radius, owner.Velocity.Cast2D(), AvoidNeighborAheadTime, CheckHitObstacleTime);
                for (var i = 0; i < neighbors.Count; ++i)
                {
                    var neighbor = neighbors[i] as MovableEntity;
                    if (null == neighbor || owner.ID == neighbor.ID)
                        continue;
                    if (!NeedAvoidNeighbor(owner, neighbor))
                        continue;

                    var add = _avoidQuerySystem.AddCircle(neighbor.ID, neighbor.Position.Cast2D(), neighbor.Radius, neighbor.Velocity.Cast2D());
                    if (add)
                        _avoidNeghborIDs.Add(neighbor.ID);
                }

                var avoidDirection = _avoidQuerySystem.QueryAvoidDirection(owner, entityManager.FrameNo, ref _avoidNeighborInfo);
                var lateral = Vector3Helpers.PerpendicularComponent(avoidDirection, owner.Forward);
                if (lateral != FixMath.F64Vec3.Zero)
                {
                    collisionAvoidance = FixMath.F64Vec3.NormalizeFast(lateral) * owner.MaxForce * Weight;
                }
            }
            return collisionAvoidance;
        }

        public override void Reset()
        {
            _avoidNeghborIDs.Clear();
            _avoidNeighborInfo.Reset();
        }

        public override void DrawGizmos(MovableEntity owner)
        {
            var annotation = owner.Annotation;
            var entityManager = owner.EntityManager;

            // draw avoid info
            if (_avoidNeighborInfo.EntityId != UniqueId.InvalidID)
            {
                var neighbor = entityManager.GetEntityById(_avoidNeighborInfo.EntityId) as MovableEntity;
                if (null != neighbor)
                {
                    var d = neighbor.Radius.Float * 2;
                    var boxSize = FixMath.F64Vec3.FromFloat(d, 0.1f, d);
                    Draw.drawBoxOutline(annotation, neighbor.Position, FixMath.F64Quat.Identity, boxSize, Colors.OrangeRed, FixMath.F64.One);
                }
            }

            // draw avoid neighbor detail info
            {
                _avoidQuerySystem.DebugDrawGizmos(owner, annotation);

                var boxSize = FixMath.F64Vec3.FromFloat(0.1f, 1.0f, 0.1f);
                for (var i = 0; i < _avoidNeghborIDs.Count; ++i)
                {
                    var neighbor = entityManager.GetEntityById(_avoidNeghborIDs[i]) as MovableEntity;
                    if (neighbor == null)
                        continue;
                    Draw.drawBoxOutline(annotation, neighbor.Position, FixMath.F64Quat.Identity, boxSize, Colors.Red, FixMath.F64.One);
                }
            }

            // draw watch neighbor
            var neighbors = owner.Neighbors;
            for (var i = 0; i < neighbors.Count; ++i)
            {
                if (Debug.IsWatchingNeighborIndex(neighbors.Count, i))
                {
                    var boxSize = FixMath.F64Vec3.FromFloat(0.1f, 2.0f, 0.1f);
                    Draw.drawBoxOutline(annotation, neighbors[i].Position, FixMath.F64Quat.Identity, boxSize, Colors.Green, FixMath.F64.One);
                }
            }
        }

        public void UpdateAvoidNeighborInfo(MovableEntity owner, IVehicle.FAvoidObstacleInfo referenceAvoidObstacleInfo)
        {
            var entityManager = owner.EntityManager;

            if (!_avoidNeighborInfo.EntityId.IsValid())
            {
                _avoidNeighborInfo.Reset();
            }

            // 远离避让单位一定距离了才考虑清除避让信息
            var other = entityManager.GetEntityById(_avoidNeighborInfo.EntityId);
            if (null == other)
            {
                _avoidNeighborInfo.Reset();
            }
            else
            {
                var distance = FixMath.F64Vec3.DistanceFast(owner.Position, other.GetPosition());
                if (distance > CheckResetAvoidInfoDistance)
                {
                    _avoidNeighborInfo.Reset();
                }
            }

            // 根据传入的FAvoidObstacleInfo信息更新避让初始信息，防止从避让Obstacle过渡到Entity方向不一致导致单位原地打转
            if (!_avoidNeighborInfo.IsValid && 
                referenceAvoidObstacleInfo.IsValid && 
                referenceAvoidObstacleInfo.FrameNo > _avoidNeighborInfo.FrameNo)
            { 
                _avoidNeighborInfo.SetAvoidInfo(entityManager.FrameNo, UniqueId.InvalidID, referenceAvoidObstacleInfo.Side);
            }
        }
    }
}
