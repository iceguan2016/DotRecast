// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System.Numerics;

namespace SharpSteer2
{
	public abstract class BaseVehicle : LocalSpace, IVehicle
	{
		public abstract FixMath.F64 Mass { get; set; }
		public abstract FixMath.F64 Radius { get; set; }
        public abstract FixMath.F64Vec3 Velocity { get; }
		public abstract FixMath.F64Vec3 Acceleration { get; }
		public abstract FixMath.F64 Speed { get; set; }

        public abstract FixMath.F64Vec3 PredictFuturePosition(FixMath.F64 predictionTime);

		public abstract FixMath.F64 MaxForce { get; }
		public abstract FixMath.F64 MaxSpeed { get; }

        public abstract FixMath.F64Vec3 GetAvoidObstacleDirection(ref Obstacles.PathIntersection pathIntersection);
        public abstract FixMath.F64Vec3 GetAvoidNeighborDirection(IVehicle threat, Obstacles.PathIntersection? intersection, ref IVehicle.FAvoidNeighborInfo info);
        public abstract bool ShouldAvoidNeighbor(IVehicle threat);

        public abstract void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision);
        public abstract void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision, SharpSteer2.Obstacles.PathIntersection nearest);
        public abstract void AnnotationAvoidCloseNeighbor(IVehicle other, FixMath.F64 additionalDistance);
        public abstract void AnnotationAvoidCloseNeighbor(IVehicle threat, FixMath.F64Vec3 avoidDirection, IVehicle.FAvoidNeighborInfo info);
        public abstract void AnnotationAvoidNeighbor(IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture);
        public abstract void AnnotationAvoidNeighbor(IVehicle threat, Obstacles.PathIntersection intersection);
    }
}
