
using System.Collections.Generic;
using System;
using FixMath;
using Pathfinding.Util;

namespace Pathfinding.Crowds
{
    public class TUnMovableEntityTemplate : TEntityTemplate
    {
        public FixMath.F64Vec2 DirU = FixMath.F64Vec2.FromInt(1, 0);
        public FixMath.F64Vec2 DirV = FixMath.F64Vec2.FromInt(0, 1);
        public FixMath.F64Vec2 HalfExtent = FixMath.F64Vec2.One;
    }

    public class UnMovableEntity : ICrowdEntityActor
    {
        public TEntityTemplate Template { get; set; }
        public IMovableEntityManager EntityManager { get; set; }
        public UniqueId ID { get; set; }

        public Volatile.VoltBody PhysicsBody { get { return _physicsBody; } }

        private FixMath.F64Vec3 _poistion;
        private FixMath.F64Quat _rotation;
        private Volatile.VoltBody _physicsBody;

        public UnMovableEntity(IMovableEntityManager entityManager)
        {
            EntityManager = entityManager;
        }

        public F64Vec3 GetPosition()
        {
            return _poistion;
        }

        public F64Quat GetRotation()
        {
            return _rotation;
        }

        public void OnCreate()
        {
            
        }

        public void OnCreatePhysicsState()
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

                _physicsBody = ICrowdEntityActor.CreatePhysicsBody(this, new Volatile.VoltShape[] { PhysicsShape });
            }
        }

        public void OnDelete()
        {
            
        }

        public void OnDestroyPhysicsState()
        {
            ICrowdEntityActor.DestroyPhysicsBody(this, _physicsBody);
        }

        public void OnPostPhysics()
        {
        }

        public void OnPrePhysics()
        {
        }

        public void OnUpdate(F64 inDeltaTime)
        {
        }

        public void SetPosition(F64Vec3 position)
        {
            _poistion = position;
        }

        public void SetRotation(F64Quat rotation)
        {
            _rotation = rotation;
        }

        bool GetBoundarySegements(FixMath.F64 InNavgationRadius, out List<FixMath.F64Vec2> OutPoints, out List<Tuple<int, int>> OutSegments)
        {
            var template = Template as TUnMovableEntityTemplate;
            var DirU = template.DirU;
            var DirV = template.DirV;
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
