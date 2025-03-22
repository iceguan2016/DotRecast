
using FixMath;
using Pathfinding.Util;

namespace Pathfinding.Crowds
{
    public class TUnMovableEntityTemplate : TEntityTemplate
    {
        
    }
    public class UnMovableEntity : ICrowdEntityActor
    {
        public TEntityTemplate Template { get; set; }
        public IMovableEntityManager EntityManager { get; set; }
        public UniqueId ID { get; set; }

        private FixMath.F64Vec3 _poistion;
        private FixMath.F64Quat _rotation;

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
        }

        public void OnDelete()
        {
        }

        public void OnDestroyPhysicsState()
        {
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
    }
}
