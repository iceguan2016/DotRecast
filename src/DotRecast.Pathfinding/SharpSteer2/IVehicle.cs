// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using Pathfinding.Util;
using SharpSteer2.Obstacles;

namespace SharpSteer2
{
	public interface IVehicle : ILocalSpaceBasis
	{
        /// <summary>
        /// mass (defaults to unity so acceleration=force)
        /// </summary>
		FixMath.F64 Mass { get; }

        /// <summary>
        /// size of bounding sphere, for obstacle avoidance, etc.
        /// </summary>
		FixMath.F64 Radius { get; }

        /// <summary>
        /// velocity of vehicle
        /// </summary>
        FixMath.F64Vec3 Velocity { get; }

		/// <summary>
		/// Gets the acceleration of the vehicle.
		/// </summary>
		FixMath.F64Vec3 Acceleration { get; }
		
		/// <summary>
        /// speed of vehicle (may be faster than taking magnitude of velocity)
		/// </summary>
		FixMath.F64 Speed { get; }

        /// <summary>
        /// predict position of this vehicle at some time in the future (assumes velocity remains constant)
        /// </summary>
        /// <param name="predictionTime"></param>
        /// <returns></returns>
        FixMath.F64Vec3 PredictFuturePosition(FixMath.F64 predictionTime);

        /// <summary>
        /// the maximum steering force this vehicle can apply
        /// </summary>
		FixMath.F64 MaxForce { get; }

        /// <summary>
        /// the maximum speed this vehicle is allowed to move
        /// </summary>
		FixMath.F64 MaxSpeed { get; }

        public enum eAvoidSide
        {
            None = 0,
            Left,
            Right
        }
        public struct FAvoidObstacleInfo
        {
            // 最新更新帧号
            public int FrameNo { get; private set; }
            // 当前避让的阻挡物
            public IObstacle Obstacle { get; private set; }
            // 当前选择的避让Obstacle选择的方向
            public eAvoidSide Side { get; private set; }

            public bool IsValid 
            {
                get 
                {
                    return Obstacle != null && Side != eAvoidSide.None;
                }
            }

            public void SetAvoidInfo(int frameNo, IObstacle obstacle, eAvoidSide side)
            {
                FrameNo = frameNo;
                Obstacle = obstacle;
                Side = side;
            }

            public void Reset()
            {
                FrameNo = 0;
                Obstacle = null;
                Side = eAvoidSide.None;
            }
        }
        FixMath.F64Vec3 GetAvoidObstacleDirection(IObstacle obstacle, ref PathIntersection pathIntersection, ref FAvoidObstacleInfo info);
        public struct FAvoidNeighborInfo
        {
            // 最新更新帧号
            public int FrameNo { get; private set; }
            // 当前避让的Entity
            public UniqueId EntityId { get; private set; }
            // 当前选择的避让Entity的VO方向
            public eAvoidSide Side { get; private set; }

            public bool IsValid
            {
                get
                {
                    return EntityId != UniqueId.InvalidID && Side != eAvoidSide.None;
                }
            }

            public void SetAvoidInfo(int frameNo, UniqueId entityId, eAvoidSide side)
            {
                FrameNo = frameNo;
                EntityId = entityId;
                Side = side;
            }

            public void Reset()
            {
                FrameNo = 0;
                EntityId = UniqueId.InvalidID;
                Side = eAvoidSide.None;
            }
        }
        FixMath.F64Vec3 GetAvoidNeighborDirection(IVehicle threat, PathIntersection? intersection, ref FAvoidNeighborInfo info);
        // returns true if the threat entity needs to be avoided.
        bool ShouldAvoidNeighbor(IVehicle threat);
        void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision);
        void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision, PathIntersection nearest);
        void AnnotationAvoidCloseNeighbor(IVehicle other, FixMath.F64 additionalDistance);
        void AnnotationAvoidCloseNeighbor(IVehicle threat, FixMath.F64Vec3 avoidDirection, IVehicle.FAvoidNeighborInfo info);
        void AnnotationAvoidNeighbor(IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture);
        void AnnotationAvoidNeighbor(IVehicle threat, PathIntersection intersection);
        void AnnotationPathFollowing(FixMath.F64Vec3 future, FixMath.F64Vec3 onPath, FixMath.F64Vec3 target, FixMath.F64 outside);
    }
}
