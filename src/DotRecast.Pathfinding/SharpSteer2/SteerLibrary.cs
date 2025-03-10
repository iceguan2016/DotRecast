// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System.Collections.Generic;
using System.Numerics;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using SharpSteer2.Pathway;

namespace SharpSteer2
{
	public abstract class SteerLibrary : BaseVehicle
	{
	    protected IAnnotationService annotation { get; private set; }

	    // Constructor: initializes state
	    protected SteerLibrary(IAnnotationService annotationService = null)
		{
            annotation = annotationService ?? new NullAnnotationService();

			// set inital state
			Reset();
		}

		// reset state
		public virtual void Reset()
		{
			// initial state of wander behavior
			_wanderSide = FixMath.F64.Zero;
			_wanderUp = FixMath.F64.Zero;
		}

        #region steering behaviours
	    private FixMath.F64 _wanderSide;
	    private FixMath.F64 _wanderUp;
	    protected FixMath.F64Vec3 SteerForWander(FixMath.F64 dt)
	    {
	        return this.SteerForWander(dt, ref _wanderSide, ref _wanderUp);
	    }

	    protected FixMath.F64Vec3 SteerForFlee(FixMath.F64Vec3 target)
	    {
	        return this.SteerForFlee(target, MaxSpeed);
	    }

	    protected FixMath.F64Vec3 SteerForSeek(FixMath.F64Vec3 target)
	    {
	        return this.SteerForSeek(target, MaxSpeed);
		}

        protected FixMath.F64Vec3 SteerForArrival(FixMath.F64Vec3 target, FixMath.F64 slowingDistance)
	    {
	        return this.SteerForArrival(target, MaxSpeed, slowingDistance, annotation);
	    }

	    protected FixMath.F64Vec3 SteerToFollowFlowField(IFlowField field, FixMath.F64 predictionTime)
	    {
	        return this.SteerToFollowFlowField(field, MaxSpeed, predictionTime, annotation);
	    }

        protected FixMath.F64Vec3 SteerToFollowPath(bool direction, FixMath.F64 predictionTime, IPathway path)
	    {
	        return this.SteerToFollowPath(direction, predictionTime, path, MaxSpeed, annotation);
	    }

        protected FixMath.F64Vec3 SteerToStayOnPath(FixMath.F64 predictionTime, IPathway path)
	    {
	        return this.SteerToStayOnPath(predictionTime, path, MaxSpeed, annotation);
	    }

        protected FixMath.F64Vec3 SteerToAvoidObstacle(FixMath.F64 minTimeToCollision, IObstacle obstacle)
        {
            return this.SteerToAvoidObstacle(minTimeToCollision, obstacle, annotation);
        }

        public struct AvoidReferenceInfo
        {
            // Record the most recent avoidance direction
            public FixMath.F64Vec3? referenceDirection;
        }
	    protected FixMath.F64Vec3 SteerToAvoidObstacles(FixMath.F64 minTimeToCollision, IEnumerable<IObstacle> obstacles, ref AvoidReferenceInfo referenceInfo)
	    {
	        return this.SteerToAvoidObstacles(minTimeToCollision, obstacles, ref referenceInfo, annotation);
	    }

	    protected FixMath.F64Vec3 SteerToAvoidNeighbors(FixMath.F64 minTimeToCollision, IEnumerable<IVehicle> others, ref IVehicle.FAvoidNeighborInfo info)
		{
	        return this.SteerToAvoidNeighbors(minTimeToCollision, others, ref info, annotation);
	    }

	    protected FixMath.F64Vec3 SteerToAvoidCloseNeighbors<TVehicle>(FixMath.F64 minSeparationDistance, IEnumerable<TVehicle> others) where TVehicle : IVehicle
        {
            return this.SteerToAvoidCloseNeighbors<TVehicle>(minSeparationDistance, others, annotation);
        }

	    protected FixMath.F64Vec3 SteerForSeparation(FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> flock)
	    {
	        return this.SteerForSeparation(maxDistance, cosMaxAngle, flock, annotation);
	    }

	    protected FixMath.F64Vec3 SteerForAlignment(FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> flock)
	    {
	        return this.SteerForAlignment(maxDistance, cosMaxAngle, flock, annotation);
	    }

	    protected FixMath.F64Vec3 SteerForCohesion(FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> flock)
	    {
	        return this.SteerForCohesion(maxDistance, cosMaxAngle, flock, annotation);
	    }

	    protected FixMath.F64Vec3 SteerForPursuit(IVehicle quarry, FixMath.F64 maxPredictionTime /*= float.MaxValue*/)
	    {
	        return this.SteerForPursuit(quarry, maxPredictionTime, MaxSpeed, annotation);
	    }

        protected FixMath.F64Vec3 SteerForEvasion(IVehicle menace, FixMath.F64 maxPredictionTime)
        {
            return this.SteerForEvasion(menace, maxPredictionTime, MaxSpeed, annotation);
        }

	    protected FixMath.F64Vec3 SteerForTargetSpeed(FixMath.F64 targetSpeed)
	    {
	        return this.SteerForTargetSpeed(targetSpeed, MaxForce, annotation);
	    }
        #endregion
	}
}
