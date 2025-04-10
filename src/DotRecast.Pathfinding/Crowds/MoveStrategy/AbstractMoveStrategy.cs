
using FixMath;
using Pathfinding.Crowds.AvoidStrategy;
using Pathfinding.Crowds.SteeringForce;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Helpers;

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

        static FixMath.F64 TICK_DELATTIME = FixMath.F64.FromDouble(0.1);
        // 预测Vechile应用steerForce，在time时间后的位置，朝向，速度，是否需要避让邻近的Idle单位
        // 主要处理Vechile一帧避让邻近单位，下一帧不避让，然后沿着路径走，会出现Vechile朝向来回摆动问题
        // 如果PredictionAvoidIdleNeighborsFoce返回非零的避让力，说明下一帧会产生避让则不应用跟随路径的力
        public virtual FixMath.F64Vec3 PredictionAvoidIdleNeighborsFoce(
            MovableEntity owner, FixMath.F64Vec3 steerForce, FixMath.F64 time, FixMath.F64 avoidNeighborAheadTime)
        {
            // 分子步执行，防止time过大问题
            steerForce = steerForce.SetYtoZero();

            var movable = new SimpleVehicle();
            movable.Position = owner.Position;
            movable.Radius = owner.Radius;
            movable.Forward = owner.Forward;
            movable.Side = owner.Side;
            movable.Up = owner.Up;
            movable.Mass = owner.Mass;
            movable.Speed = owner.Speed;
            movable.MaxSpeed = owner.MaxSpeed;
            movable.MaxForce = owner.MaxForce;

            var avoidQuerySystem = new AvoidanceQuerySystem();

            var restTime = time;
            do 
            {
                var deltaTime = restTime > TICK_DELATTIME? TICK_DELATTIME : restTime;

                // 1. 预测位置
                movable.ApplySteeringForce(steerForce, time);

                // 2.预测避让信息
                var neighbors = owner.Neighbors;
                avoidQuerySystem.Init(owner.ID, movable.Position.Cast2D(), movable.Radius, movable.Velocity.Cast2D(), avoidNeighborAheadTime, false);
                for (var i = 0; i < neighbors.Count; ++i)
                {
                    var neighbor = neighbors[i] as MovableEntity;
                    if (null == neighbor || owner.ID == neighbor.ID)
                        continue;
                    if (!AbstractSteeringForce.CheckGroupShouldAvoid(owner, neighbor))
                        continue;
                    if (neighbor.HasEntityState(MovableEntity.eEntityState.Moving))
                        continue;

                    avoidQuerySystem.AddCircle(neighbor.ID, neighbor.Position.Cast2D(), neighbor.Radius, neighbor.Velocity.Cast2D());
                }

                var avoidNeighborInfo = new IVehicle.FAvoidNeighborInfo();
                var avoidDirection = avoidQuerySystem.QueryAvoidDirection(owner, owner.EntityManager.FrameNo, ref avoidNeighborInfo);
                if (avoidDirection != F64Vec3.Zero)
                {
                    return avoidDirection;
                }

                restTime -= deltaTime;
            } while (restTime > 0);
            return FixMath.F64Vec3.Zero;
        }

        public abstract void OnTemplateChanged(MovableEntity owner);

        public virtual void DrawGizmos(MovableEntity owner)
        {
            
        }
    }
}
