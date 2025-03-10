using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using SharpSteer2.Obstacles;
using SharpSteer2.Pathway;

namespace SharpSteer2.Helpers
{
    public static class VehicleHelpers
    {
        public static FixMath.F64Vec3 SteerForWander(this IVehicle vehicle, FixMath.F64 dt, ref FixMath.F64 wanderSide, ref FixMath.F64 wanderUp, IAnnotationService annotation = null)
        {
            // random walk WanderSide and WanderUp between -1 and +1
            var speed = 12 * dt; // maybe this (12) should be an argument?
            wanderSide = Utilities.ScalarRandomWalk(wanderSide, speed, -FixMath.F64.One, FixMath.F64.One);
            wanderUp = Utilities.ScalarRandomWalk(wanderUp, speed, -FixMath.F64.One, FixMath.F64.One);

            // return a pure lateral steering vector: (+/-Side) + (+/-Up)
            return (vehicle.Side * wanderSide) + (vehicle.Up * wanderUp);
        }

        public static FixMath.F64Vec3 SteerForFlee(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            var offset = vehicle.Position - target;
            var desiredVelocity = offset.TruncateLength(maxSpeed); //xxxnew
            return desiredVelocity - vehicle.Velocity;
        }

        public static FixMath.F64Vec3 SteerForSeek(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            var offset = target - vehicle.Position;
            var desiredVelocity = offset.TruncateLength(maxSpeed); //xxxnew
            return desiredVelocity - vehicle.Velocity;
        }

        public static FixMath.F64Vec3 SteerForArrival(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 maxSpeed, FixMath.F64 slowingDistance, IAnnotationService annotation = null)
        {
            var offset = target - vehicle.Position;
            var distance = FixMath.F64Vec3.LengthFast(offset);
            var rampedSpeed = maxSpeed * (distance / slowingDistance);
            var clippedSpeed = FixMath.F64.Min(rampedSpeed, maxSpeed);
            var desiredVelocity = (clippedSpeed / distance) * offset;
            return desiredVelocity - vehicle.Velocity;
        }

        public static FixMath.F64Vec3 SteerToFollowFlowField(this IVehicle vehicle, IFlowField flowField, FixMath.F64 maxSpeed, FixMath.F64 predictionDistance, IAnnotationService annotation = null)
        {
            var futurePosition = vehicle.PredictFuturePosition(predictionDistance);
            var flow = flowField.Sample(futurePosition);
            return vehicle.Velocity - flow.TruncateLength(maxSpeed);
        }

        public static FixMath.F64Vec3 SteerToStayOnPath(this IVehicle vehicle, FixMath.F64 predictionTime, IPathway path, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            // predict our future position
            var futurePosition = vehicle.PredictFuturePosition(predictionTime);

            // find the point on the path nearest the predicted future position
            FixMath.F64Vec3 tangent;
            FixMath.F64 outside;
            FixMath.F64Vec3 onPath = path.MapPointToPath(futurePosition, out tangent, out outside);

            if (outside < 0)
                return FixMath.F64Vec3.Zero;    // our predicted future position was in the path, return zero steering.

            // our predicted future position was outside the path, need to
            // steer towards it.  Use onPath projection of futurePosition
            // as seek target
            if (annotation != null)
                annotation.PathFollowing(vehicle, futurePosition, onPath, onPath, outside);

            return vehicle.SteerForSeek(onPath, maxSpeed);
        }

        public static FixMath.F64Vec3 SteerToFollowPath(this IVehicle vehicle, bool direction, FixMath.F64 predictionTime, IPathway path, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            FixMath.F64 pathDistance;
            return SteerToFollowPath(vehicle, direction, predictionTime, path, maxSpeed, out pathDistance, annotation);
        }

        public static FixMath.F64Vec3 SteerToFollowPath(this IVehicle vehicle, bool direction, FixMath.F64 predictionTime, IPathway path, FixMath.F64 maxSpeed, out FixMath.F64 currentPathDistance, IAnnotationService annotation = null)
        {
            // our goal will be offset from our path distance by this amount
            var pathDistanceOffset = (direction ? 1 : -1) * predictionTime * vehicle.Speed;

            // predict our future position
            var futurePosition = vehicle.PredictFuturePosition(predictionTime);

            // measure distance along path of our current and predicted positions
            currentPathDistance = path.MapPointToPathDistance(vehicle.Position);
            var futurePathDistance = path.MapPointToPathDistance(futurePosition);

            // are we facing in the correction direction?
            bool rightway = ((pathDistanceOffset > 0) ?
                            (currentPathDistance < futurePathDistance) :
                            (currentPathDistance > futurePathDistance));

            // find the point on the path nearest the predicted future position
            FixMath.F64Vec3 tangent;
            FixMath.F64 outside;
            FixMath.F64Vec3 onPath = path.MapPointToPath(futurePosition, out tangent, out outside);

            // no steering is required if (a) our future position is inside
            // the path tube and (b) we are facing in the correct direction
            if ((outside <= 0) && rightway)
            {
                //We're going at max speed, in the right direction. don't need to do anything
                if (vehicle.Speed >= maxSpeed)
                    return FixMath.F64Vec3.Zero;

                //Predict vehicle position and sample multiple times, incresingly far along the path
                var seek = path.MapPointToPath(vehicle.PredictFuturePosition(predictionTime / 3), out tangent, out outside);
                for (int i = 0; i < 3; i++)
                {
                    var s = path.MapPointToPath(seek + tangent * vehicle.Speed / FixMath.F64.FromInt(i + 1), out tangent, out outside);

                    //terminate search if we wander outside the path
                    if (outside > 0)
                        break;
                    seek = s;

                    if (annotation != null)
                        annotation.Circle3D(FixMath.F64.FromFloat(0.3f), seek, FixMath.F64Vec3.AxisX, Colors.Green, 6);
                }

                //Steer towards future path point
                return vehicle.SteerForSeek(seek, maxSpeed, annotation);
            }

            // otherwise we need to steer towards a target point obtained
            // by adding pathDistanceOffset to our current path position
            var targetPathDistance = currentPathDistance + pathDistanceOffset;
            var target = path.MapPathDistanceToPoint(targetPathDistance);

            if (annotation != null)
                annotation.PathFollowing(vehicle, futurePosition, onPath, target, outside);

            // return steering to seek target on path
            return SteerForSeek(vehicle, target, maxSpeed);
        }

        /// <summary>
        /// Returns a steering force to avoid a given obstacle.  The purely
        /// lateral steering force will turn our this towards a silhouette edge
        /// of the obstacle.  Avoidance is required when (1) the obstacle
        /// intersects the this's current path, (2) it is in front of the
        /// this, and (3) is within minTimeToCollision seconds of travel at the
        /// this's current velocity.  Returns a zero vector value (Vector3::zero)
        /// when no avoidance is required.
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="minTimeToCollision"></param>
        /// <param name="obstacle"></param>
        /// <param name="annotation"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 SteerToAvoidObstacle(this IVehicle vehicle, FixMath.F64 minTimeToCollision, IObstacle obstacle, IAnnotationService annotation = null)
        {
            var referenceInfo = new SteerLibrary.AvoidReferenceInfo();
            var avoidance = obstacle.steerToAvoid(vehicle as BaseVehicle, minTimeToCollision, ref referenceInfo);

            // XXX more annotation modularity problems (assumes spherical obstacle)
            if (avoidance != FixMath.F64Vec3.Zero && annotation != null)
                annotation.AvoidObstacle(vehicle, minTimeToCollision * vehicle.Speed);

            return avoidance;
        }

        public static FixMath.F64Vec3 SteerToAvoidObstacles(this IVehicle vehicle, FixMath.F64 minTimeToCollision, IEnumerable<IObstacle> obstacles, ref SteerLibrary.AvoidReferenceInfo referenceInfo, IAnnotationService annotation = null)
        {
            var avoidance = Obstacle.steerToAvoidObstacles(vehicle as BaseVehicle, 
                                                           minTimeToCollision,
                                                           obstacles,
                                                           out var nearest,
                                                           ref referenceInfo);

            // XXX more annotation modularity problems (assumes spherical obstacle)
            if (annotation != null && avoidance != FixMath.F64Vec3.Zero)
                annotation.AvoidObstacle(vehicle, minTimeToCollision * vehicle.Speed, nearest);

            return avoidance;
        }

        public static FixMath.F64Vec3 SteerForSeparation(this IVehicle vehicle, FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> others, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighbors = FixMath.F64.Zero;

            // for each of the other vehicles...
            foreach (var other in others)
            {
                if (!IsInBoidNeighborhood(vehicle, other, vehicle.Radius * 3, maxDistance, cosMaxAngle))
                    continue;

                // add in steering contribution
                // (opposite of the offset direction, divided once by distance
                // to normalize, divided another time to get 1/d falloff)
                var offset = other.Position - vehicle.Position;
                var distanceSquared = FixMath.F64Vec3.Dot(offset, offset);
                steering += (offset / -distanceSquared);

                // count neighbors
                neighbors++;
            }

            // divide by neighbors, then normalize to pure direction
            if (neighbors > 0)
            {
                steering = FixMath.F64Vec3.NormalizeFast(steering / neighbors);
            }

            return steering;
        }

        /// <summary>
        /// avoidance of "close neighbors"
        /// </summary>
        /// <remarks>
        /// Does a hard steer away from any other agent who comes withing a
        /// critical distance.  Ideally this should be replaced with a call
        /// to steerForSeparation.
        /// </remarks>
        /// <typeparam name="TVehicle"></typeparam>
        /// <param name="vehicle"></param>
        /// <param name="minSeparationDistance"></param>
        /// <param name="others"></param>
        /// <param name="annotation"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 SteerToAvoidCloseNeighbors<TVehicle>(this IVehicle vehicle, FixMath.F64 minSeparationDistance, IEnumerable<TVehicle> others, IAnnotationService annotation = null)
            where TVehicle : IVehicle
        {
            // for each of the other vehicles...
            foreach (IVehicle other in others)
            {
                if (other != vehicle)
                {
                    var sumOfRadii = vehicle.Radius + other.Radius;
                    var minCenterToCenter = minSeparationDistance + sumOfRadii;
                    var offset = other.Position - vehicle.Position;
                    var currentDistance = FixMath.F64Vec3.LengthFast(offset);

                    if (currentDistance < minCenterToCenter)
                    {
                        if (annotation != null)
                            annotation.AvoidCloseNeighbor(vehicle, other, minSeparationDistance);

                        return Vector3Helpers.PerpendicularComponent(-offset, vehicle.Forward);
                    }
                }
            }

            // otherwise return zero
            return FixMath.F64Vec3.Zero;
        }

        public static FixMath.F64Vec3 SteerForAlignment(this IVehicle vehicle, FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> flock, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighbors = FixMath.F64.Zero;

            // for each of the other vehicles...
            foreach (IVehicle other in flock.Where(other => vehicle.IsInBoidNeighborhood(other, vehicle.Radius * 3, maxDistance, cosMaxAngle)))
            {
                // accumulate sum of neighbor's heading
                steering += other.Forward;

                // count neighbors
                neighbors++;
            }

            // divide by neighbors, subtract off current heading to get error-
            // correcting direction, then normalize to pure direction
            if (neighbors > 0)
            {
                steering = ((steering / neighbors) - vehicle.Forward);

                var length = FixMath.F64Vec3.LengthFast(steering);
                if (length > FixMath.F64.FromFloat(0.025f))
                    steering /= length;
            }

            return steering;
        }

        public static FixMath.F64Vec3 SteerForCohesion(this IVehicle vehicle, FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle, IEnumerable<IVehicle> flock, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighbors = FixMath.F64.Zero;

            // for each of the other vehicles...
            foreach (IVehicle other in flock.Where(other => vehicle.IsInBoidNeighborhood(other, vehicle.Radius * 3, maxDistance, cosMaxAngle)))
            {
                // accumulate sum of neighbor's positions
                steering += other.Position;

                // count neighbors
                neighbors++;
            }

            // divide by neighbors, subtract off current position to get error-
            // correcting direction, then normalize to pure direction
            if (neighbors > 0)
            {
                steering = FixMath.F64Vec3.NormalizeFast((steering / neighbors) - vehicle.Position);
            }

            return steering;
        }

        private readonly static FixMath.F64[,] _pursuitFactors = new FixMath.F64[3, 3]
        {
            { FixMath.F64.FromFloat(2), FixMath.F64.FromFloat(2), FixMath.F64.FromFloat(0.5f) },         //Behind
            { FixMath.F64.FromFloat(4), FixMath.F64.FromFloat(0.8f), FixMath.F64.FromFloat(1) },         //Aside
            { FixMath.F64.FromFloat(0.85f), FixMath.F64.FromFloat(1.8f), FixMath.F64.FromFloat(4) },     //Ahead
        };
        
        static FixMath.F64 MinSpeedValue = FixMath.F64.FromFloat(0.001f);
        static FixMath.F64 MinAngleCosValue = FixMath.F64.FromFloat(-0.707f);
        static FixMath.F64 MaxAngleCosValue = FixMath.F64.FromFloat(0.707f);
        public static FixMath.F64Vec3 SteerForPursuit(this IVehicle vehicle, IVehicle quarry, FixMath.F64 maxPredictionTime, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            // offset from this to quarry, that distance, unit vector toward quarry
            var offset = quarry.Position - vehicle.Position;
            var distance = FixMath.F64Vec3.LengthFast(offset);
            var unitOffset = offset / distance;

            // how parallel are the paths of "this" and the quarry
            // (1 means parallel, 0 is pependicular, -1 is anti-parallel)
            var parallelness = FixMath.F64Vec3.Dot(vehicle.Forward, quarry.Forward);

            // how "forward" is the direction to the quarry
            // (1 means dead ahead, 0 is directly to the side, -1 is straight back)
            var forwardness = FixMath.F64Vec3.Dot(vehicle.Forward, unitOffset);

            var directTravelTime = distance / FixMath.F64.Max(MinSpeedValue, vehicle.Speed);
            int f = Utilities.IntervalComparison(forwardness, MinAngleCosValue, MaxAngleCosValue);
            int p = Utilities.IntervalComparison(parallelness, MinAngleCosValue, MaxAngleCosValue);

            // Break the pursuit into nine cases, the cross product of the
            // quarry being [ahead, aside, or behind] us and heading
            // [parallel, perpendicular, or anti-parallel] to us.
            var timeFactor = _pursuitFactors[f + 1, p + 1];

            // estimated time until intercept of quarry
            var et = directTravelTime * timeFactor;

            // xxx experiment, if kept, this limit should be an argument
            var etl = (et > maxPredictionTime) ? maxPredictionTime : et;

            // estimated position of quarry at intercept
            var target = quarry.PredictFuturePosition(etl);

            // annotation
            if (annotation != null)
                annotation.Line(vehicle.Position, target, Colors.DarkGray, FixMath.F64.One);

            return SteerForSeek(vehicle, target, maxSpeed, annotation);
        }

        public static FixMath.F64Vec3 SteerForEvasion(this IVehicle vehicle, IVehicle menace, FixMath.F64 maxPredictionTime, FixMath.F64 maxSpeed, IAnnotationService annotation = null)
        {
            // offset from this to menace, that distance, unit vector toward menace
            var offset = menace.Position - vehicle.Position;
            var distance = FixMath.F64Vec3.LengthFast(offset);

            var roughTime = distance / menace.Speed;
            var predictionTime = ((roughTime > maxPredictionTime) ? maxPredictionTime : roughTime);

            var target = menace.PredictFuturePosition(predictionTime);

            return SteerForFlee(vehicle, target, maxSpeed, annotation);
        }

        /// <summary>
        /// tries to maintain a given speed, returns a maxForce-clipped steering
        /// force along the forward/backward axis
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="targetSpeed"></param>
        /// <param name="maxForce"></param>
        /// <param name="annotation"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 SteerForTargetSpeed(this IVehicle vehicle, FixMath.F64 targetSpeed, FixMath.F64 maxForce, IAnnotationService annotation = null)
        {
            var mf = maxForce;
            var speedError = targetSpeed - vehicle.Speed;
            return vehicle.Forward * Utilities.Clamp(speedError, -mf, mf);
        }

        /// <summary>
        /// Unaligned collision avoidance behavior: avoid colliding with other
        /// nearby vehicles moving in unconstrained directions.  Determine which
        /// (if any) other other this we would collide with first, then steers
        /// to avoid the site of that potential collision.  Returns a steering
        /// force vector, which is zero length if there is no impending collision.
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="minTimeToCollision"></param>
        /// <param name="others"></param>
        /// <param name="annotation"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 SteerToAvoidNeighbors(this IVehicle vehicle, FixMath.F64 minTimeToCollision, IEnumerable<IVehicle> others, IAnnotationService annotation = null)
        {
            // first priority is to prevent immediate interpenetration
            var separation = SteerToAvoidCloseNeighbors(vehicle, FixMath.F64.Zero, others, annotation);
            if (separation != FixMath.F64Vec3.Zero)
                return separation;

            // otherwise, go on to consider potential future collisions
            var steer = FixMath.F64.Zero;
            IVehicle threat = null;

            // Time (in seconds) until the most immediate collision threat found
            // so far.  Initial value is a threshold: don't look more than this
            // many frames into the future.
            var minTime = minTimeToCollision;

            // xxx solely for annotation
            var xxxThreatPositionAtNearestApproach = FixMath.F64Vec3.Zero;
            var xxxOurPositionAtNearestApproach = FixMath.F64Vec3.Zero;

            // for each of the other vehicles, determine which (if any)
            // pose the most immediate threat of collision.
            foreach (IVehicle other in others)
            {
                if (other != vehicle)
                {
                    // avoid when future positions are this close (or less)
                    var collisionDangerThreshold = vehicle.Radius * 2;

                    // predicted time until nearest approach of "this" and "other"
                    var time = PredictNearestApproachTime(vehicle, other);

                    // If the time is in the future, sooner than any other
                    // threatened collision...
                    if ((time >= 0) && (time < minTime))
                    {
                        // if the two will be close enough to collide,
                        // make a note of it
                        if (ComputeNearestApproachPositions(vehicle, other, time, out var ourPositionAtNearestApproach, out var hisPositionAtNearestApproach) 
                            < collisionDangerThreshold)
                        {
                            minTime = time;
                            threat = other;
                        }
                    }
                }
            }

            // if a potential collision was found, compute steering to avoid
            if (threat != null)
            {
                // parallel: +1, perpendicular: 0, anti-parallel: -1
                var parallelness = FixMath.F64Vec3.Dot(vehicle.Forward, threat.Forward);
                var ANGLE = FixMath.F64.FromFloat(0.707f);

                if (parallelness < -ANGLE)
                {
                    // anti-parallel "head on" paths:
                    // steer away from future threat position
                    var offset = xxxThreatPositionAtNearestApproach - vehicle.Position;
                    var sideDot = FixMath.F64Vec3.Dot(offset, vehicle.Side);
                    steer = (sideDot > FixMath.F64.Zero) ? -FixMath.F64.One : FixMath.F64.One;
                }
                else
                {
                    if (parallelness > ANGLE)
                    {
                        // parallel paths: steer away from threat
                        var offset = threat.Position - vehicle.Position;
                        var sideDot = FixMath.F64Vec3.Dot(offset, vehicle.Side);
                        steer = (sideDot > FixMath.F64.Zero) ? -FixMath.F64.One : FixMath.F64.One;
                    }
                    else
                    {
                        // perpendicular paths: steer behind threat
                        // (only the slower of the two does this)
                        if (threat.Speed <= vehicle.Speed)
                        {
                            var sideDot = FixMath.F64Vec3.Dot(vehicle.Side, threat.Velocity);
                            steer = (sideDot > FixMath.F64.Zero) ? -FixMath.F64.One : FixMath.F64.One;
                        }
                    }
                }

                if (annotation != null)
                    annotation.AvoidNeighbor(vehicle, threat, steer, xxxOurPositionAtNearestApproach, xxxThreatPositionAtNearestApproach);
            }

            return vehicle.Side * steer;
        }

        public static FixMath.F64Vec3 SteerToAvoidNeighbors(this IVehicle vehicle, FixMath.F64 minTimeToCollision, IEnumerable<IVehicle> others, ref IVehicle.FAvoidNeighborInfo info, IAnnotationService annotation = null)
        {
            // https://www.jdxdev.com/blog/2021/03/19/boids-for-rts/
            // https://howtorts.github.io/2014/01/14/avoidance-behaviours.html
            // 

            System.Func<IVehicle, FixMath.F64Vec3> avoidThreatEntity = threat =>
            {
                var resultVector = FixMath.F64Vec3.Zero;
                var obstacle = new SphericalObstacle(threat.Radius, threat.Position);
                PathIntersection intersection = PathIntersection.DEFAULT;
                obstacle.findIntersectionWithVehiclePath(vehicle as BaseVehicle, ref intersection);
                if (intersection.intersect)
                {
                    // Choose right direction
                    resultVector = new FixMath.F64Vec3(intersection.surfaceNormal.Z, FixMath.F64.Zero, -intersection.surfaceNormal.X);
                    resultVector.Normalize();

                    //Move it out based on our radius + theirs
                    resultVector *= vehicle.Radius + threat.Radius;

                    if (null != annotation) annotation.AvoidNeighbor(vehicle, threat, intersection);
                }
                return resultVector;
            };

            // Find min distance entity
            var minSeparationDistance = FixMath.F64.Zero;
            foreach (var other in others)
            {
                if (other != vehicle)
                {
                    var sumOfRadii = vehicle.Radius + other.Radius;
                    var minCenterToCenter = minSeparationDistance + sumOfRadii;
                    var offset = other.Position - vehicle.Position;
                    var currentDistance = offset.Length();

                    if (currentDistance < minCenterToCenter)
                    {
                        return avoidThreatEntity(other);
                    }
                }
            }

            // otherwise, go on to consider potential future collisions
            IVehicle threat = null;

            // Time (in seconds) until the most immediate collision threat found
            // so far.  Initial value is a threshold: don't look more than this
            // many frames into the future.
            var minTime = minTimeToCollision;

            // xxx solely for annotation
            var xxxThreatPositionAtNearestApproach = FixMath.F64Vec3.Zero;
            var xxxOurPositionAtNearestApproach = FixMath.F64Vec3.Zero;

            // for each of the other vehicles, determine which (if any)
            // pose the most immediate threat of collision.
            foreach (var other in others)
            {
                if (other != vehicle)
                {
                    // avoid when future positions are this close (or less)
                    var collisionDangerThreshold = vehicle.Radius * 2;

                    // predicted time until nearest approach of "this" and "other"
                    var time = PredictNearestApproachTime(vehicle, other);

                    // If the time is in the future, sooner than any other
                    // threatened collision...
                    if ((time >= 0) && (time < minTime))
                    {
                        // if the two will be close enough to collide,
                        // make a note of it
                        if (ComputeNearestApproachPositions(vehicle, other, time, out var ourPositionAtNearestApproach, out var hisPositionAtNearestApproach)
                            < collisionDangerThreshold)
                        {
                            minTime = time;
                            threat = other;
                            xxxThreatPositionAtNearestApproach
                                = hisPositionAtNearestApproach?? FixMath.F64Vec3.Zero;
                            xxxOurPositionAtNearestApproach
                                = ourPositionAtNearestApproach?? FixMath.F64Vec3.Zero;
                        }
                    }
                }
            }

            // if a potential collision was found, compute steering to avoid
            if (threat != null)
            {
                // return avoidThreatEntity(threat);
                var avoidDir = vehicle.GetAvoidNeighborDirection(threat, null, ref info);
                return vehicle.Velocity - avoidDir.TruncateLength(vehicle.MaxSpeed);
            }

            // otherwise return zero
            return FixMath.F64Vec3.Zero;
        }

        /// <summary>
        /// Given two vehicles, based on their current positions and velocities,
        /// determine the time until nearest approach
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        private static FixMath.F64 PredictNearestApproachTime(IVehicle vehicle, IVehicle other)
        {
            // imagine we are at the origin with no velocity,
            // compute the relative velocity of the other this
            var myVelocity = vehicle.Velocity;
            var otherVelocity = other.Velocity;
            var relVelocity = otherVelocity - myVelocity;
            var relSpeed = FixMath.F64Vec3.LengthFast(relVelocity);

            // for parallel paths, the vehicles will always be at the same distance,
            // so return 0 (aka "now") since "there is no time like the present"
            if (FixMath.F64.Abs(relSpeed) < FixMath.F64.Epsilon)
                return FixMath.F64.Zero;

            // Now consider the path of the other this in this relative
            // space, a line defined by the relative position and velocity.
            // The distance from the origin (our this) to that line is
            // the nearest approach.

            // Take the unit tangent along the other this's path
            var relTangent = relVelocity / relSpeed;

            // find distance from its path to origin (compute offset from
            // other to us, find length of projection onto path)
            var relPosition = vehicle.Position - other.Position;
            var projection = FixMath.F64Vec3.Dot(relTangent, relPosition);

            return projection / relSpeed;
        }

        /// <summary>
        /// Given the time until nearest approach (predictNearestApproachTime)
        /// determine position of each this at that time, and the distance
        /// between them
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="other"></param>
        /// <param name="time"></param>
        /// <returns></returns>
        private static FixMath.F64 ComputeNearestApproachPositions(IVehicle vehicle, IVehicle other, FixMath.F64 time, 
            out FixMath.F64Vec3? ourPositionAtNearestApproach, out FixMath.F64Vec3? hisPositionAtNearestApproach)
        {
            var myTravel = vehicle.Forward * vehicle.Speed * time;
            var otherTravel = other.Forward * other.Speed * time;

            var myFinal = vehicle.Position + myTravel;
            var otherFinal = other.Position + otherTravel;

            // xxx for annotation
            ourPositionAtNearestApproach = myFinal;
            hisPositionAtNearestApproach = otherFinal;

            return FixMath.F64Vec3.DistanceFast(myFinal, otherFinal);
        }

        public static bool IsAhead(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 cosThreshold /*= 0.707f*/)
        {
            var targetDirection = FixMath.F64Vec3.NormalizeFast(target - vehicle.Position);
            return FixMath.F64Vec3.Dot(vehicle.Forward, targetDirection) > cosThreshold;
        }

        public static bool IsAside(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 cosThreshold /*= 0.707f*/)
        {
            var targetDirection = FixMath.F64Vec3.NormalizeFast(target - vehicle.Position);
            var dp = FixMath.F64Vec3.Dot(vehicle.Forward, targetDirection);
            return (dp < cosThreshold) && (dp > -cosThreshold);
        }

        public static bool IsBehind(this IVehicle vehicle, FixMath.F64Vec3 target, FixMath.F64 cosThreshold /*= -0.707f*/)
        {
            var targetDirection = FixMath.F64Vec3.NormalizeFast(target - vehicle.Position);
            return FixMath.F64Vec3.Dot(vehicle.Forward, targetDirection) < cosThreshold;
        }

        private static bool IsInBoidNeighborhood(this ILocalSpaceBasis vehicle, ILocalSpaceBasis other, FixMath.F64 minDistance, FixMath.F64 maxDistance, FixMath.F64 cosMaxAngle)
        {
            if (other == vehicle)
                return false;

            var offset = other.Position - vehicle.Position;
            var distanceSquared = FixMath.F64Vec3.LengthSqr(offset);

            // definitely in neighborhood if inside minDistance sphere
            if (distanceSquared < (minDistance * minDistance))
                return true;

            // definitely not in neighborhood if outside maxDistance sphere
            if (distanceSquared > (maxDistance * maxDistance))
                return false;

            // otherwise, test angular offset from forward axis
            var unitOffset = offset / FixMath.F64.SqrtFast(distanceSquared);
            var forwardness = FixMath.F64Vec3.Dot(vehicle.Forward, unitOffset);
            return forwardness > cosMaxAngle;
        }
    }
}
