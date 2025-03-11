// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using DotRecast.Pathfinding.Util;
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

        // calculate reference avoidance direction
        FixMath.F64Vec3 GetAvoidObstacleDirection(ref PathIntersection pathIntersection);
        public enum eAvoidVOSide
        {
            None = 0,
            Left,
            Right
        }
        public struct FAvoidNeighborInfo
        {
            // 当前避让的Entity
            public UniqueId EntityId { get; set; }
            // 当前选择的避让Entity的VO方向
            public eAvoidVOSide VOSide { get; set; }
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
    }
}
