
using FixMath;
using Pathfinding.Util;
using Volatile;

namespace Pathfinding.Crowds
{
    public abstract class AbstractCrowdEntity : ICrowdEntityActor
    {
        public TEntityTemplate Template { get; set; }
        public IMovableEntityManager EntityManager { get; set; }
        public UniqueId ID { get; set; }
        public VoltBody PhysicsBody { get; set; }
        public int GroupMask { get; set; }
        public int GroupsToAvoid { get; set; }

        public abstract F64Vec3 GetPosition();
        public abstract F64Quat GetRotation();
        public abstract void SetPosition(F64Vec3 position);
        public abstract void SetRotation(F64Quat rotation);
        public abstract void OnCreate();
        public abstract void OnCreatePhysicsState();
        public abstract void OnDelete();
        public abstract void OnDestroyPhysicsState();
        public abstract void OnPostPhysics();
        public abstract void OnPrePhysics();
        public abstract void OnUpdate(F64 inDeltaTime);
    }
}
