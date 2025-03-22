
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
            return null;
        }

        public static void DestroyPhysicsBody(ICrowdEntityActor entity, Volatile.VoltBody body)
        {
            
        }
    }
}
