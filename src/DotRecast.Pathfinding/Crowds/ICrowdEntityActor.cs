
using System.Drawing;
using Pathfinding.Util;

namespace Pathfinding.Crowds
{
    public class TEntityTemplate
    {
        public FixMath.F64 Density = FixMath.F64.FromFloat(1.0f);
    }

    public interface ICrowdEntityActor
    {
        public TEntityTemplate Template { get; set; }

        public IMovableEntityManager EntityManager { get; set; }
        public UniqueId ID { get; set; }
        public Volatile.VoltBody PhysicsBody { get; }

        void SetPosition(FixMath.F64Vec3 position);
        FixMath.F64Vec3 GetPosition();
        void SetRotation(FixMath.F64Quat rotation);
        FixMath.F64Quat GetRotation();

        void OnCreate();
        void OnDelete();
        void OnCreatePhysicsState();
        void OnDestroyPhysicsState();
        void OnPrePhysics();
        void OnPostPhysics();

        void OnUpdate(FixMath.F64 inDeltaTime);

        public static Volatile.VoltBody CreatePhysicsBody(ICrowdEntityActor entity, params Volatile.VoltShape[] shapesToAdd)
        {
            var physicsWorld = entity.EntityManager.PhysicsWorld;
            if (null == physicsWorld)
                return null;

            var position = entity.GetPosition().ToVoltVec2();
            FixMath.F64Quat.ToEulerAnglesRad(entity.GetRotation(), out var euler);
            var angle = euler.Y.ToF64();
            // 创建Body
            Volatile.VoltBody body = null;
            if (entity is UnMovableEntity)
            {
                body = physicsWorld.CreateStaticBody(position, angle, shapesToAdd);
            }
            else if (entity is MovableEntity)
            {
                body = physicsWorld.CreateDynamicBody(position, angle, shapesToAdd);
            }
            if (null != body) body.UserData = entity;
            return body;
        }

        public static void DestroyPhysicsBody(ICrowdEntityActor entity, Volatile.VoltBody body)
        {
            var physicsWorld = entity.EntityManager.PhysicsWorld;
            if (null == physicsWorld)
                return;

            if (null != body)
            {
                body.UserData = null;
                physicsWorld.RemoveBody(body);
            }
        }
    }
}
