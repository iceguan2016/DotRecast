// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System;
using System.Numerics;
using SharpSteer2.Helpers;

namespace SharpSteer2
{
	public class SimpleVehicle : SteerLibrary
	{
	    FixMath.F64Vec3 _lastForward;
        FixMath.F64Vec3 _lastPosition;
		FixMath.F64 _smoothedCurvature;
		// The acceleration is smoothed
        FixMath.F64Vec3 _acceleration;

		public SimpleVehicle(IAnnotationService annotations = null)
            : base(annotations)
		{
		}

		// reset vehicle state
		public override void Reset()
		{
            base.Reset();

			// reset LocalSpace state
			ResetLocalSpace();

			Mass = FixMath.F64.One;          // Mass (defaults to 1 so acceleration=force)
			Speed = FixMath.F64.Zero;         // speed along Forward direction.

			Radius = FixMath.F64.Half;     // size of bounding sphere

			// reset bookkeeping to do running averages of these quanities
			ResetSmoothedPosition();
			ResetSmoothedCurvature(FixMath.F64.Zero);
			ResetAcceleration();
		}

		// get/set Mass
        // Mass (defaults to unity so acceleration=force)
	    public override FixMath.F64 Mass { get; set; }

	    // get velocity of vehicle
        public override FixMath.F64Vec3 Velocity
		{
			get { return Forward * Speed; }
		}

		// get/set speed of vehicle  (may be faster than taking mag of velocity)
        // speed along Forward direction. Because local space is
        // velocity-aligned, velocity = Forward * Speed
	    public override FixMath.F64 Speed { get; set; }

	    // size of bounding sphere, for obstacle avoidance, etc.
	    public override FixMath.F64 Radius { get; set; }

	    // get/set maxForce
        // the maximum steering force this vehicle can apply
        // (steering force is clipped to this magnitude)
		static FixMath.F64 DefaultMaxForce = FixMath.F64.FromFloat(0.1f);
        public override FixMath.F64 MaxForce
        {
            get { return DefaultMaxForce; }
        }

	    // get/set maxSpeed
        // the maximum speed this vehicle is allowed to move
        // (velocity is clipped to this magnitude)
		static FixMath.F64 DefaultMaxSpeed = FixMath.F64.FromFloat(1.0f);
	    public override FixMath.F64 MaxSpeed
	    {
	        get { return DefaultMaxSpeed; }
	    }

	    // apply a given steering force to our momentum,
		// adjusting our orientation to maintain velocity-alignment.
	    public void ApplySteeringForce(FixMath.F64Vec3 force, FixMath.F64 elapsedTime)
		{
			var adjustedForce = AdjustRawSteeringForce(force, elapsedTime);

			// enforce limit on magnitude of steering force
            var clippedForce = adjustedForce.TruncateLength(MaxForce);

			// compute acceleration and velocity
			var newAcceleration = (clippedForce / Mass);
			var newVelocity = Velocity;

			// damp out abrupt changes and oscillations in steering acceleration
			// (rate is proportional to time step, then clipped into useful range)
			if (elapsedTime > 0)
			{
                var smoothRate = Utilities.Clamp(FixMath.F64.FromInt(9) * elapsedTime, FixMath.F64.FromFloat(0.15f), FixMath.F64.FromFloat(0.4f));
				Utilities.BlendIntoAccumulator(smoothRate, newAcceleration, ref _acceleration);
			}

			// Euler integrate (per frame) acceleration into velocity
			newVelocity += _acceleration * elapsedTime;

			// enforce speed limit
            newVelocity = newVelocity.TruncateLength(MaxSpeed);

			// update Speed
			Speed = (FixMath.F64Vec3.LengthFast(newVelocity));

			// Euler integrate (per frame) velocity into position
			Position = (Position + (newVelocity * elapsedTime));

			// regenerate local space (by default: align vehicle's forward axis with
			// new velocity, but this behavior may be overridden by derived classes.)
			RegenerateLocalSpace(newVelocity, elapsedTime);

			// maintain path curvature information
			MeasurePathCurvature(elapsedTime);

			// running average of recent positions
			Utilities.BlendIntoAccumulator(elapsedTime * FixMath.F64.FromFloat(0.06f), // QQQ
								  Position,
								  ref _smoothedPosition);
		}

		// the default version: keep FORWARD parallel to velocity, change
		// UP as little as possible.
	    protected virtual void RegenerateLocalSpace(FixMath.F64Vec3 newVelocity, FixMath.F64 elapsedTime)
		{
			// adjust orthonormal basis vectors to be aligned with new velocity
			if (Speed > 0)
			{
				RegenerateOrthonormalBasisUF(newVelocity / Speed);
			}
		}

		// alternate version: keep FORWARD parallel to velocity, adjust UP
		// according to a no-basis-in-reality "banking" behavior, something
		// like what birds and airplanes do.  (XXX experimental cwr 6-5-03)
	    protected void RegenerateLocalSpaceForBanking(FixMath.F64Vec3 newVelocity, FixMath.F64 elapsedTime)
		{
			// the length of this global-upward-pointing vector controls the vehicle's
			// tendency to right itself as it is rolled over from turning acceleration
			var globalUp = FixMath.F64Vec3.FromFloat(0, 0.2f, 0);

			// acceleration points toward the center of local path curvature, the
			// length determines how much the vehicle will roll while turning
			var accelUp = _acceleration * FixMath.F64.FromFloat(0.05f);

			// combined banking, sum of UP due to turning and global UP
			var bankUp = accelUp + globalUp;

			// blend bankUp into vehicle's UP basis vector
			var smoothRate = elapsedTime * 3;
			var tempUp = Up;
			Utilities.BlendIntoAccumulator(smoothRate, bankUp, ref tempUp);
			Up = FixMath.F64Vec3.NormalizeFast(tempUp);

			var AxisScale = FixMath.F64.FromInt(4);
            annotation.Line(Position, Position + (globalUp * AxisScale), Colors.White, FixMath.F64.One);
	        annotation.Line(Position, Position + (bankUp * AxisScale), Colors.Orange, FixMath.F64.One);
			annotation.Line(Position, Position + (accelUp * AxisScale), Colors.Red, FixMath.F64.One);
	        annotation.Line(Position, Position + (Up * FixMath.F64.One), Colors.Gold, FixMath.F64.One);

			// adjust orthonormal basis vectors to be aligned with new velocity
			if (Speed > 0) RegenerateOrthonormalBasisUF(newVelocity / Speed);
		}

		/// <summary>
        /// adjust the steering force passed to applySteeringForce.
        /// allows a specific vehicle class to redefine this adjustment.
        /// default is to disallow backward-facing steering at low speed.
		/// </summary>
		/// <param name="force"></param>
		/// <param name="deltaTime"></param>
		/// <returns></returns>
		protected virtual FixMath.F64Vec3 AdjustRawSteeringForce(FixMath.F64Vec3 force, FixMath.F64 deltaTime)
		{
			var maxAdjustedSpeed = FixMath.F64.FromFloat(0.2f) * MaxSpeed;

			if ((Speed > maxAdjustedSpeed) || (force == FixMath.F64Vec3.Zero))
				return force;

            var range = Speed / maxAdjustedSpeed;
            var cosine = Utilities.Lerp(FixMath.F64.One, -FixMath.F64.One, FixMath.F64.Pow(range, FixMath.F64.FromFloat(20)));
            return force.LimitMaxDeviationAngle(cosine, Forward);
		}

		/// <summary>
        /// apply a given braking force (for a given dt) to our momentum.
		/// </summary>
		/// <param name="rate"></param>
		/// <param name="deltaTime"></param>
	    public void ApplyBrakingForce(FixMath.F64 rate, FixMath.F64 deltaTime)
		{
			var rawBraking = Speed * rate;
			var clipBraking = ((rawBraking < MaxForce) ? rawBraking : MaxForce);
			Speed = (Speed - (clipBraking * deltaTime));
		}

		/// <summary>
        /// predict position of this vehicle at some time in the future (assumes velocity remains constant)
		/// </summary>
		/// <param name="predictionTime"></param>
		/// <returns></returns>
        public override FixMath.F64Vec3 PredictFuturePosition(FixMath.F64 predictionTime)
		{
			return Position + (Velocity * predictionTime);
		}

		// get instantaneous curvature (since last update)
	    protected FixMath.F64 Curvature { get; private set; }

	    // get/reset smoothedCurvature, smoothedAcceleration and smoothedPosition
		public FixMath.F64 SmoothedCurvature
		{
			get { return _smoothedCurvature; }
		}

	    private void ResetSmoothedCurvature(FixMath.F64 value /*= 0*/)
		{
			_lastForward = FixMath.F64Vec3.Zero;
			_lastPosition = FixMath.F64Vec3.Zero;
	        _smoothedCurvature = value;
            Curvature = value;
		}

		public override FixMath.F64Vec3 Acceleration
		{
			get { return _acceleration; }
		}

	    protected void ResetAcceleration()
	    {
	        ResetAcceleration(FixMath.F64Vec3.Zero);
	    }

	    private void ResetAcceleration(FixMath.F64Vec3 value)
	    {
	        _acceleration = value;
	    }

        FixMath.F64Vec3 _smoothedPosition;
	    public FixMath.F64Vec3 SmoothedPosition
		{
			get { return _smoothedPosition; }
		}

	    private void ResetSmoothedPosition()
	    {
	        ResetSmoothedPosition(FixMath.F64Vec3.Zero);
	    }

	    protected void ResetSmoothedPosition(FixMath.F64Vec3 value)
	    {
	        _smoothedPosition = value;
	    }

	    // set a random "2D" heading: set local Up to global Y, then effectively
		// rotate about it by a random angle (pick random forward, derive side).
	    protected void RandomizeHeadingOnXZPlane()
		{
			Up = FixMath.F64Vec3.AxisY;
            Forward = Vector3Helpers.RandomUnitVectorOnXZPlane();
	        Side = FixMath.F64Vec3.Cross(Forward, Up);
		}

		// measure path curvature (1/turning-radius), maintain smoothed version
		void MeasurePathCurvature(FixMath.F64 elapsedTime)
		{
			if (elapsedTime > 0)
			{
				var dP = _lastPosition - Position;
				var dF = (_lastForward - Forward) / FixMath.F64Vec3.LengthFast(dP);
                var lateral = Vector3Helpers.PerpendicularComponent(dF, Forward);
                var sign = (FixMath.F64Vec3.Dot(lateral, Side) < 0) ? FixMath.F64.One : -FixMath.F64.One;
				Curvature = FixMath.F64Vec3.LengthFast(lateral) * sign;
				Utilities.BlendIntoAccumulator(elapsedTime * FixMath.F64.FromFloat(4.0f), Curvature, ref _smoothedCurvature);
				_lastForward = Forward;
				_lastPosition = Position;
			}
		}
	}
}
