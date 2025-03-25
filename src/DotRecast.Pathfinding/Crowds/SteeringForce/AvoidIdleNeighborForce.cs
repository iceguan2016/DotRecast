

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

        private AvoidanceQuerySystem _avoidQuerySystem = new AvoidanceQuerySystem();
        private List<UniqueId> _avoidNeghborIDs = new List<UniqueId>();

        private IVehicle.FAvoidNeighborInfo _avoidNeighborInfo = new IVehicle.FAvoidNeighborInfo();

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
            updateAvoidNeighborInfo(owner);

            var entityManager = owner.EntityManager;
            var neighbors = owner.Neighbors;
            var collisionAvoidance = FixMath.F64Vec3.Zero;
            {
                _avoidNeghborIDs.Clear();
                _avoidQuerySystem.Init(owner.ID, owner.Position.Cast2D(), owner.Radius, owner.Velocity.Cast2D(), AvoidNeighborAheadTime);
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
                collisionAvoidance = FixMath.F64Vec3.NormalizeFast(lateral) * owner.MaxForce * Weight;
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

            // 6. draw avoid neighbor detail info
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
        }

        void updateAvoidNeighborInfo(MovableEntity owner)
        {
            var entityManager = owner.EntityManager;

            if (!_avoidNeighborInfo.EntityId.IsValid())
            {
                _avoidNeighborInfo.Reset();
                return;
            }

            // 超过一定帧数没有需要避让的单位，则重置避让信息
            var deltaFrame = entityManager.FrameNo - _avoidNeighborInfo.FrameNo;
            if (deltaFrame > 5)
            {
                _avoidNeighborInfo.Reset();
            }
        }
    }
}
