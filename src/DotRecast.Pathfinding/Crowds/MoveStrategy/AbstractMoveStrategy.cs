
using FixMath;
using Pathfinding.Crowds.AvoidStrategy;

namespace Pathfinding.Crowds.MoveStrategy
{
    public abstract class AbstractMoveStrategy : IMoveStrategy
    {
        public abstract bool Condition(MovableEntity owner);

        public virtual void OnOverlapBegin(MovableEntity owner, MovableEntity other)
        {
        }

        public virtual void OnOverlapEnd(MovableEntity owner, MovableEntity other)
        {
        }

        public abstract void OnStart(MovableEntity owner);

        public abstract void OnStop(MovableEntity owner);

        public abstract F64Vec3 OnUpdate(MovableEntity owner, F64 deltaSeconds);

        public virtual void DrawGizmos(MovableEntity owner)
        {
            
        }
    }
}
