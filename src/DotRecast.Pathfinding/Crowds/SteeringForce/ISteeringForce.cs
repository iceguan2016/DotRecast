using System;
using System.Collections.Generic;
using System.Text;
using Pathfinding.Crowds;
using SharpSteer2;

namespace Pathfinding.Crowds.SteeringForce
{
    // 避让策略接口类，策略分几个等级，优先级由高到低：
    // 1.被同阵营优先级更高的单位推开（自己处于Idle状态，可以被推挤开，避让完毕后要回到原先位置）
    // 2.避让正在避让自己的单位（对向单位正在避让自己，则临时选择和其相同的避让Side，达到相互避让效果）
    // 3.避让Idle状态下的单位（处于Idle状态的邻近单位，且不能推挤开）
    // 4.避让Wall（避让场景中的约束边，防止进入阻挡物中）

    public interface ISteeringForce
    {
        // force weight
        FixMath.F64 Weight { get; set; }
        // calculate final steering force
        FixMath.F64Vec3 GetSteeringForce(MovableEntity owner);
        // draw debug gizmos
        void DrawGizmos(MovableEntity owner);
    }
}
