
using System.Collections.Generic;
using System;
using FixMath;
using Pathfinding.Util;
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Crowds
{
    public class TUnMovableEntityTemplate : TEntityTemplate
    {
        public FixMath.F64Vec2 HalfExtent = FixMath.F64Vec2.One;

        public override int GetHashCode()
        {
            return Density.GetHashCode()
                ^ HalfExtent.GetHashCode();
        }
    }

    public class UnMovableEntity : AbstractCrowdEntity
    {
        private FixMath.F64Vec3 _poistion;
        private FixMath.F64Quat _rotation;
        private Obstacle        _obstacle;

        public UnMovableEntity(IMovableEntityManager entityManager)
        {
            EntityManager = entityManager;
        }

        public override F64Vec3 GetPosition() { return _poistion; }
        public override F64Quat GetRotation() { return _rotation; }
        public override void SetPosition(F64Vec3 position) { _poistion = position; }
        public override void SetRotation(F64Quat rotation) { _rotation = rotation; }

        public override void OnCreate()
        {
            var template = Template as TUnMovableEntityTemplate;
            // 添加到Map
            var map = EntityManager.Map;
            _obstacle = map.AddObstacle(ID, _poistion, _rotation, template.HalfExtent.Cast(FixMath.F64.Zero));
        }

        public override void OnCreatePhysicsState()
        {
            var PhysicsWorld = EntityManager.PhysicsWorld;
            if (null == PhysicsWorld)
            {
                return;
            }

            var template = Template as TUnMovableEntityTemplate;

            if (GetBoundarySegements(FixMath.F64.Zero, out var OutPoints, out var OutSegments))
            {
                Volatile.VoltVector2[] Vertices = new Volatile.VoltVector2[OutPoints.Count];
                // 这里要保证边的法线向外，否则检测不到碰撞，可以把Volt引擎的GizmoDraw打开查看绘制信息
                //for (int i = 0; i < OutPoints.Count; i++)
                //    Vertices[i] = OutPoints[i].ToVoltVec2();
                for (int i = OutPoints.Count - 1, j = 0; i >= 0; i--, j++)
                    Vertices[j] = OutPoints[i].ToVoltVec2();
                var PhysicsShape = PhysicsWorld.CreatePolygonWorldSpace(Vertices, template.Density.ToF64());

                PhysicsBody = ICrowdEntityActor.CreatePhysicsBody(this, new Volatile.VoltShape[] { PhysicsShape });
            }
        }

        public override void OnDelete()
        {
            if (null != _obstacle)
            {
                var map = EntityManager.Map;
                map.RemoveObstacle(_obstacle);
            }
        }

        public override void OnDestroyPhysicsState()
        {
            ICrowdEntityActor.DestroyPhysicsBody(this, PhysicsBody);
        }

        public override void OnPostPhysics()
        {
        }

        public override void OnPrePhysics()
        {
        }

        public override void OnUpdate(F64 inDeltaTime)
        {
        }

        bool GetBoundarySegements(FixMath.F64 InNavgationRadius, out List<FixMath.F64Vec2> OutPoints, out List<Tuple<int, int>> OutSegments)
        {
            var template = Template as TUnMovableEntityTemplate;
            var xAxis = _rotation * FixMath.F64Vec3.AxisX;
            var zAxis = _rotation * FixMath.F64Vec3.AxisZ;
            var DirU = xAxis.Cast2D();
            var DirV = zAxis.Cast2D();
            var HalfExtent = template.HalfExtent;

            OutPoints = new List<FixMath.F64Vec2>();
            OutSegments = new List<Tuple<int, int>>();

            var Center = _poistion.Cast2D();
            var HalfExtentForNavgation = HalfExtent + FixMath.F64Vec2.One * InNavgationRadius;

            OutPoints.Add(Center + DirU * HalfExtentForNavgation.X + DirV * HalfExtentForNavgation.Y);
            OutPoints.Add(Center - DirU * HalfExtentForNavgation.X + DirV * HalfExtentForNavgation.Y);
            OutPoints.Add(Center - DirU * HalfExtentForNavgation.X - DirV * HalfExtentForNavgation.Y);
            OutPoints.Add(Center + DirU * HalfExtentForNavgation.X - DirV * HalfExtentForNavgation.Y);

            OutSegments.Add(Tuple.Create(0, 1));
            OutSegments.Add(Tuple.Create(1, 2));
            OutSegments.Add(Tuple.Create(2, 3));
            OutSegments.Add(Tuple.Create(3, 0));

            //OutSegments.Add(Tuple.Create(1, 0));
            //OutSegments.Add(Tuple.Create(2, 1));
            //OutSegments.Add(Tuple.Create(3, 2));
            //OutSegments.Add(Tuple.Create(0, 3));

            return true;
        }
    }
}
