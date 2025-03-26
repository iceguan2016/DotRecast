using System.Collections.Generic;
using SharpSteer2.Helpers;
using Pathfinding.Util;
using SharpSteer2;

namespace Pathfinding.Crowds
{
    public class AvoidanceQuerySystem
    {
        protected UniqueId OwnerEntityId { get; set; }
        protected FixMath.F64Vec2 Position { get; set; }
        protected FixMath.F64 Radius { get; set; }
        protected FixMath.F64Vec2 Velocity { get; set; }

        protected FixMath.F64 timeHorizon = FixMath.F64.Half;
        protected FixMath.F64 invTimeHorizon = FixMath.F64.One;

        public class VO
        {
            public static readonly int EdgeLeftIndex = 0;
            public static readonly int EdgeRightIndex = 1;
            public struct VOEdge 
            {
                // Edge所属的Entity
                public UniqueId Owner { get; private set; }
                // Edge方向
                public FixMath.F64Vec2 Direction { get; private set; }
                // 角度[-Pi~Pi)
                public FixMath.F64 Angle { get; private set; }
                //
                public FixMath.F64 DistanceSquared { get; private set; }

                public VOEdge(UniqueId id, FixMath.F64Vec2 dir, FixMath.F64 distSq)
                {
                    Owner = id;
                    Direction = dir;
                    if (dir != FixMath.F64Vec2.Zero)
                    {
                        Angle = FixMath.F64.Atan2Fastest(dir.Y, dir.X); // (-PI, PI)
                        // while (Angle < 0) Angle += FixMath.F64.Pi2;
                        // while (Angle > FixMath.F64.Pi2) Angle -= FixMath.F64.Pi2;
                    }
                    else
                    {
                        Angle = FixMath.F64.Zero;
                    }
                    DistanceSquared = distSq;
                }

                public void SetEdge(UniqueId id, FixMath.F64Vec2 dir, FixMath.F64 distSq)
                {
                    Owner = id;
                    Direction = dir;
                    if (dir != FixMath.F64Vec2.Zero)
                    {
                        Angle = FixMath.F64.Atan2Fastest(dir.Y, dir.X); // (-PI, PI)
                        // while (Angle < 0) Angle += FixMath.F64.Pi2;
                        // while (Angle > FixMath.F64.Pi2) Angle -= FixMath.F64.Pi2;
                    }
                    else
                    {
                        Angle = FixMath.F64.Zero;
                    }
                    DistanceSquared = distSq;
                }

                public void SetEdge(UniqueId id, FixMath.F64Vec2 dir, FixMath.F64 angle, FixMath.F64 distSq)
                {
                    Owner = id;
                    Direction = dir;
                    Angle = angle;
                    DistanceSquared = distSq;
                }
            }

            public VOEdge[] Edges = new VOEdge[2] 
            {
                new VOEdge(UniqueId.InvalidID, FixMath.F64Vec2.Zero, FixMath.F64.Zero),
                new VOEdge(UniqueId.InvalidID, FixMath.F64Vec2.Zero, FixMath.F64.Zero)
            };

            public bool Contains(FixMath.F64 angle)
            {
                var left = Edges[EdgeLeftIndex].Angle;
                var right = Edges[EdgeRightIndex].Angle;
                // 注意：这里left - right的范围表示从left顺时针转到right所扫过的范围
                // 比如当left为-170度(-2.96弧度)，right为-20度(-0.34弧度)就会出现left < right情况，需要分开处理
                return left >= right? (left >= angle && angle >= right) : (angle <= left || angle >= right);
            }

            public bool TryMergeWith(VO other)
            {
                ref var left = ref Edges[EdgeLeftIndex];
                ref var right = ref Edges[EdgeRightIndex];
                ref var otherLeft = ref other.Edges[EdgeLeftIndex];
                ref var otherRight = ref other.Edges[EdgeRightIndex];

                bool leftContain = Contains(otherLeft.Angle);
                bool rightContain = Contains(otherRight.Angle); /* attention */

                if (leftContain && rightContain)
                {
                    return true;
                }

                if (leftContain)
                {
                    right.SetEdge(otherRight.Owner, otherRight.Direction, otherRight.Angle, otherRight.DistanceSquared);
                }

                if (rightContain)
                {
                    left.SetEdge(otherLeft.Owner, otherLeft.Direction, otherLeft.Angle, otherLeft.DistanceSquared);
                }

                return leftContain || rightContain;
            }
        }

        protected List<VO> voLists = new List<VO>();
        public List<VO> VOList
        {
            get { return voLists; }
        }

        public void Init(
            UniqueId ownerEntityId,
            FixMath.F64Vec2 position, FixMath.F64 radius,
            FixMath.F64Vec2 velocity, FixMath.F64 timeHorizon)
        {
            this.Position = position;
            this.Radius = radius;
            this.Velocity = velocity;
            this.timeHorizon = timeHorizon;
            this.invTimeHorizon = FixMath.F64.One / timeHorizon;

            this.voLists.Clear();
        }

        public bool AddCircle(
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

            // if (distSq > combinedRadiusSq)
            {
                FixMath.F64 tmin, tmax;
                bool collision = Geometry.TimeToCollisionWithCircle2D(
                    ownerPosition2D, ownerRadius, relativeVelocity,
                    otherPosition2D, otherRadius, out tmin, out tmax);
                if (collision && tmin < this.timeHorizon)
                // if (FixMath.F64Vec2.Dot(Velocity, relativePosition) > 0)  // in front
                {
                    /* will collision */
                    FixMath.F64 leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                    left = new FixMath.F64Vec2(relativePosition.X * leg - relativePosition.Y * combinedRadius,
                            relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                    right = new FixMath.F64Vec2(relativePosition.X * leg + relativePosition.Y * combinedRadius,
                            -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;

                    /* add to vo list */
                    VO tmpVO = new VO();
                    tmpVO.Edges[VO.EdgeLeftIndex].SetEdge(neighborEntityId, left, distSq);
                    tmpVO.Edges[VO.EdgeRightIndex].SetEdge(neighborEntityId, right, distSq);
                    var addTempVO = true;
                    for (int i = voLists.Count - 1; i >= 0; --i)
                    {
                        VO vo = voLists[i];
                        if (tmpVO.TryMergeWith(vo))
                        {
                            voLists.RemoveAt(i);
                        }
                        else if (vo.TryMergeWith(tmpVO))
                        {
                            addTempVO = false;
                            break;
                        }
                    }
                    if (addTempVO) voLists.Add(tmpVO);

                    debugVOs.Add((neighborEntityId, new List<VO>(voLists)));
                    return true;
                }
            }
            return false;
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
        public FixMath.F64Vec3 QueryAvoidDirection(IVehicle vehicle, int frameNo, ref IVehicle.FAvoidNeighborInfo info)
        {
            /* selected nearest velocity */
            var velDir = FixMath.F64Vec2.NormalizeFast(vehicle.Velocity.Cast2D());
            var entity = UniqueId.InvalidID;
            var minDistSq = FixMath.F64.MaxValue;
            var bestDir = FixMath.F64Vec2.Zero;
            var side = IVehicle.eAvoidSide.None;
            for (int i = 0; i < voLists.Count; ++i)
            {
                ref var left = ref voLists[i].Edges[VO.EdgeLeftIndex];
                ref var right = ref voLists[i].Edges[VO.EdgeRightIndex];             
                
                if (info.Side != IVehicle.eAvoidSide.Right)
                {
                    var diffLeft = left.Direction - velDir;
                    var diffLeftSq = FixMath.F64Vec2.LengthSqr(diffLeft);
                    if (diffLeftSq < minDistSq)
                    {
                        entity = left.Owner;
                        bestDir = left.Direction;
                        minDistSq = diffLeftSq;
                        side = IVehicle.eAvoidSide.Left;
                    }
                }
                // 
                if (info.Side != IVehicle.eAvoidSide.Left)
                {
                    var diffRight = right.Direction - velDir;
                    var diffRightSq = FixMath.F64Vec2.LengthSqr(diffRight);
                    if (diffRightSq < minDistSq)
                    {
                        entity = right.Owner;
                        bestDir = right.Direction;
                        minDistSq = diffRightSq;
                        side = IVehicle.eAvoidSide.Right;
                    }
                }
            }
            
            if (side != IVehicle.eAvoidSide.None)
            {
                info.SetAvoidInfo(frameNo, entity, side);
                return bestDir.Cast(FixMath.F64.Zero);
            }
            return FixMath.F64Vec3.Zero;
        }

        public void DebugDrawGizmos(IVehicle vehicle, IAnnotationService annotation)
        {
            var pos = vehicle.Position;

            // draw all vos
            var minLen = FixMath.F64.FromFloat(5.0f);
            var maxLen = FixMath.F64.FromFloat(20.0f);
            var stepLen = FixMath.F64.FromFloat(2.0f);
            for (var i = 0; i < voLists.Count; ++i)
            {
                var len = FixMath.F64.Max(maxLen - stepLen * i, minLen);

                ref var left = ref voLists[i].Edges[VO.EdgeLeftIndex];
                ref var right = ref voLists[i].Edges[VO.EdgeRightIndex];

                // left
                var start0 = Position;
                var end0 = Position + left.Direction * len;
                Draw.drawLine(annotation, start0.Cast(pos.Y), end0.Cast(pos.Y), Colors.Red);
                // right
                var start1 = Position;
                var end1 = Position + right.Direction * len;
                Draw.drawLine(annotation, start1.Cast(pos.Y), end1.Cast(pos.Y), Colors.Green);

                Draw.drawLine(annotation, end0.Cast(pos.Y), end1.Cast(pos.Y), Colors.Blue);
            }
        }
    }

}
