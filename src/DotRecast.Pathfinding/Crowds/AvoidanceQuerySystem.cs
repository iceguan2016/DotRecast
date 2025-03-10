using System.Collections.Generic;
using SharpSteer2.Helpers;
using Pathfinding.Util;
using DotRecast.Pathfinding.Crowds;
using DotRecast.Pathfinding.Util;

namespace Pathfinding.Crowds
{
    public class AvoidanceQuerySystem
    {
        protected UniqueId OwnerEntityId { get; set; }
        protected FixMath.F64Vec2 Position { get; set; }
        protected FixMath.F64Vec2 PrevPosition { get; set; }
        protected FixMath.F64 Radius { get; set; }
        protected FixMath.F64Vec2 Velocity { get; set; }

        protected FixMath.F64 timeHorizon = FixMath.F64.Half;
        protected FixMath.F64 invTimeHorizon = FixMath.F64.One;

        public class VO
        {
            // VO所属的Entity和距离
            public UniqueId Owner { get; set; }
            public FixMath.F64 DistanceSquared { get; set; }

            protected FixMath.F64Vec2 left = FixMath.F64Vec2.Zero;
            public FixMath.F64Vec2 Left
            {
                get { return left; }
            }
            protected FixMath.F64Vec2 right = FixMath.F64Vec2.Zero;
            public FixMath.F64Vec2 Right
            {
                get { return right; }
            }

            public VO(FixMath.F64Vec2 l, FixMath.F64Vec2 r)
            {
                left = l;
                right = r;
            }

            public bool Contains(FixMath.F64Vec2 v)
            {
                return v.Det(left) >= 0 && v.Det(right) >= 0;
            }

            public bool OverlapWith(VO other, bool merge)
            {
                return OverlapWith(other.Left, -other.Right, merge);
            }

            public void TryMergeWith(VO other)
            {
                
            }

            public bool OverlapWith(FixMath.F64Vec2 left, FixMath.F64Vec2 right, bool merge)
            {
                bool leftContain = Contains(left);
                bool rightContain = Contains(right); /* attention */
                bool overlap = (leftContain && !rightContain) ||
                    (!leftContain && rightContain);
                if (overlap && merge)
                {
                    if (leftContain)
                    {
                        this.right = right;
                    }
                    else
                    {
                        this.left = left;
                    }
                }
                return overlap;
            }
        }

        protected List<VO> voLists = new List<VO>();
        public List<VO> VOList
        {
            get { return voLists; }
        }

        public void Init(
            UniqueId ownerEntityId,
            FixMath.F64Vec2 position, FixMath.F64Vec2 prevPosition, FixMath.F64 radius,
            FixMath.F64Vec2 velocity, FixMath.F64 timeHorizon)
        {
            this.Position = position;
            this.PrevPosition = prevPosition;
            this.Radius = radius;
            this.Velocity = velocity;
            this.timeHorizon = timeHorizon;
            this.invTimeHorizon = FixMath.F64.One / timeHorizon;

            this.voLists.Clear();
        }

        public bool CheckCollisionWith(
            FixMath.F64Vec2 position, FixMath.F64 radius, FixMath.F64Vec2 velocity)
        {
            /* calculate vo */
            FixMath.F64Vec2 ownerPosition2D = this.Position;
            FixMath.F64Vec2 ownerVelocity = this.Velocity;
            FixMath.F64 ownerRadius = this.Radius;

            FixMath.F64Vec2 otherVel = velocity;
            FixMath.F64Vec2 otherPosition2D = position;
            FixMath.F64 otherRadius = radius;   /* Maybe scale other radius for better avoid performance */

            FixMath.F64Vec2 relativePosition = otherPosition2D - ownerPosition2D;
            FixMath.F64Vec2 relativeVelocity = ownerVelocity - otherVel;
            FixMath.F64 distSq = FixMath.F64Vec2.LengthSqr(relativePosition);
            FixMath.F64 combinedRadius = ownerRadius + otherRadius;
            FixMath.F64 combinedRadiusSq = combinedRadius * combinedRadius;

            FixMath.F64Vec2 u = FixMath.F64Vec2.Zero;

            if (distSq > combinedRadiusSq)
            {
                FixMath.F64 tmin, tmax;
                bool collision = Geometry.TimeToCollisionWithCircle2D(
                    ownerPosition2D, ownerRadius, relativeVelocity,
                    otherPosition2D, otherRadius, out tmin, out tmax);
                if (collision && tmin > 0 && tmin < this.timeHorizon)
                {
                    return true;
                }
                else
                {
                    /* no collision */
                    return false;
                }
            }
            else
            {
                return true;
            }
        }

        public void AddCircle(
            UniqueId neighborEntityId,
            FixMath.F64Vec2 position, 
            FixMath.F64 radius, 
            FixMath.F64Vec2 velocity)
        {
            /* calculate vo */
            FixMath.F64Vec2 ownerPosition2D = this.Position;
            FixMath.F64Vec2 ownerVelocity = this.Velocity;
            FixMath.F64 ownerRadius = this.Radius;

            FixMath.F64Vec2 otherVel = velocity;
            FixMath.F64Vec2 otherPosition2D = position;
            FixMath.F64 otherRadius = radius;   /* Maybe scale other radius for better avoid performance */

            FixMath.F64Vec2 relativePosition = otherPosition2D - ownerPosition2D;
            FixMath.F64Vec2 relativeVelocity = ownerVelocity - otherVel;
            FixMath.F64 distSq = FixMath.F64Vec2.LengthSqr(relativePosition);
            FixMath.F64 combinedRadius = ownerRadius + otherRadius;
            FixMath.F64 combinedRadiusSq = combinedRadius * combinedRadius;

            FixMath.F64Vec2 u = FixMath.F64Vec2.Zero;
            FixMath.F64Vec2 left, right;

            if (distSq > combinedRadiusSq)
            {
                FixMath.F64 tmin, tmax;
                bool collision = Geometry.TimeToCollisionWithCircle2D(
                    ownerPosition2D, ownerRadius, relativeVelocity,
                    otherPosition2D, otherRadius, out tmin, out tmax);
                if (collision && tmin > 0 && tmin < this.timeHorizon)
                {
                    /* will collision */
                    FixMath.F64 leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                    left = new FixMath.F64Vec2(relativePosition.X * leg - relativePosition.Y * combinedRadius,
                            relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                    right = -new FixMath.F64Vec2(relativePosition.X * leg + relativePosition.Y * combinedRadius,
                            -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;

                    /* add to vo list */
                    VO tmpVO = new VO(left, right);
                    tmpVO.Owner = neighborEntityId;
                    tmpVO.DistanceSquared = distSq;
                    for (int i = voLists.Count - 1; i >= 0; --i)
                    {
                        VO vo = voLists[i];
                        var canMerge = distSq < vo.DistanceSquared;
                        if (canMerge && tmpVO.OverlapWith(vo, true))
                        {
                            voLists.RemoveAt(i);
                        }
                    }
                    voLists.Add(tmpVO);
                }
                else
                {
                    /* no collision */
                    return;
                }
            }
        }

        public void AddSegement(
            FixMath.F64Vec2 start, FixMath.F64Vec2 end)
        {

        }

        // 选择最优避让方向规则：
        // (1) 与desiredVelocity方向最接近
        // (2) 和上一次选择side保持一致
        // (3) 读取避让单位是否在避让自己，选择一致的避让方向（比如都选左，或者右）
        // (4) 检查选择的避让方向是否会撞墙
        public struct FQueryBestAvoidDirectionParams
        {
            public FixMath.F64Vec2  desiredVelocity;
            public int              side;  // -1 - left, 0 - any, 1 - right
        }
        public FixMath.F64Vec2 QueryBestAvoidDirection(ref FQueryBestAvoidDirectionParams inOutParams)
        {
            /* selected nearest velocity */
            var velDir = FixMath.F64Vec2.NormalizeFast(inOutParams.desiredVelocity);
            var minDistSq = FixMath.F64.MaxValue;
            var bestDir = velDir;
            var side = 0;
            for (int i = 0; i < voLists.Count; ++i)
            {
                var left = voLists[i].Left;
                var right = voLists[i].Right;             
                
                // 当side == 1时候，只能选择右边的避让方向
                if (inOutParams.side <= 0)
                {
                    var diffLeft = left - velDir;
                    var diffLeftSq = FixMath.F64Vec2.LengthSqr(diffLeft);
                    if (diffLeftSq < minDistSq)
                    {
                        bestDir = left;
                        minDistSq = diffLeftSq;
                        side = -1;
                    }
                }
                // 当side == -1时候，只能选择左边的避让方向
                if (inOutParams.side >= 0)
                {
                    var diffRight = right - velDir;
                    var diffRightSq = FixMath.F64Vec2.LengthSqr(diffRight);
                    if (diffRightSq < minDistSq)
                    {
                        bestDir = right;
                        minDistSq = diffRightSq;
                        side = 1;
                    }
                }
            }
            if (inOutParams.side == 0)
            {
                inOutParams.side = side;
            }
            return bestDir;
        }
    }

}
