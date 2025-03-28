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
        public static Vector3 SteerForWander(this IVehicle vehicle, float dt, ref float wanderSide, ref float wanderUp, IAnnotationService annotation = null)
        {
            // random walk WanderSide and WanderUp between -1 and +1
            float speed = 12 * dt; // maybe this (12) should be an argument?
            wanderSide = Utilities.ScalarRandomWalk(wanderSide, speed, -1, +1);
            wanderUp = Utilities.ScalarRandomWalk(wanderUp, speed, -1, +1);

            // return a pure lateral steering vector: (+/-Side) + (+/-Up)
            return (vehicle.Side * wanderSide) + (vehicle.Up * wanderUp);
        }

        public static Vector3 SteerForFlee(this IVehicle vehicle, Vector3 target, float maxSpeed, IAnnotationService annotation = null)
        {
            Vector3 offset = vehicle.Position - target;
            Vector3 desiredVelocity = offset.TruncateLength(maxSpeed); //xxxnew
            return desiredVelocity - vehicle.Velocity;
        }

        public static Vector3 SteerForSeek(this IVehicle vehicle, Vector3 target, float maxSpeed, IAnnotationService annotation = null)
        {
            Vector3 offset = target - vehicle.Position;
            Vector3 desiredVelocity = offset.TruncateLength(maxSpeed); //xxxnew
            return desiredVelocity - vehicle.Velocity;
        }

        public static Vector3 SteerForArrival(this IVehicle vehicle, Vector3 target, float maxSpeed, float slowingDistance, IAnnotationService annotation = null)
        {
            Vector3 offset = target - vehicle.Position;
            float distance = offset.Length();
            float rampedSpeed = maxSpeed * (distance / slowingDistance);
            float clippedSpeed = Math.Min(rampedSpeed, maxSpeed);
            Vector3 desiredVelocity = (clippedSpeed / distance) * offset;
            return desiredVelocity - vehicle.Velocity;
        }

        public static Vector3 SteerToFollowFlowField(this IVehicle vehicle, IFlowField flowField, float maxSpeed, float predictionDistance, IAnnotationService annotation = null)
        {
            var futurePosition = vehicle.PredictFuturePosition(predictionDistance);
            var flow = flowField.Sample(futurePosition);
            return vehicle.Velocity - flow.TruncateLength(maxSpeed);
        }

        public static Vector3 SteerToStayOnPath(this IVehicle vehicle, float predictionTime, IPathway path, float maxSpeed, IAnnotationService annotation = null)
        {
            // predict our future position
            Vector3 futurePosition = vehicle.PredictFuturePosition(predictionTime);

            // find the point on the path nearest the predicted future position
            Vector3 tangent;
            float outside;
            Vector3 onPath = path.MapPointToPath(futurePosition, out tangent, out outside);

            if (outside < 0)
                return Vector3.Zero;    // our predicted future position was in the path, return zero steering.

            // our predicted future position was outside the path, need to
            // steer towards it.  Use onPath projection of futurePosition
            // as seek target
            if (annotation != null)
                annotation.PathFollowing(futurePosition, onPath, onPath, outside);

            return vehicle.SteerForSeek(onPath, maxSpeed);
        }

        public static Vector3 SteerToFollowPath(this IVehicle vehicle, bool direction, float predictionTime, IPathway path, float maxSpeed, IAnnotationService annotation = null)
        {
            float pathDistance;
            return SteerToFollowPath(vehicle, direction, predictionTime, path, maxSpeed, out pathDistance, annotation);
        }

        public static Vector3 SteerToFollowPath(this IVehicle vehicle, bool direction, float predictionTime, IPathway path, float maxSpeed, out float currentPathDistance, IAnnotationService annotation = null)
        {
            // our goal will be offset from our path distance by this amount
            float pathDistanceOffset = (direction ? 1 : -1) * predictionTime * vehicle.Speed;

            // predict our future position
            Vector3 futurePosition = vehicle.PredictFuturePosition(predictionTime);

            // measure distance along path of our current and predicted positions
            currentPathDistance = path.MapPointToPathDistance(vehicle.Position);
            float futurePathDistance = path.MapPointToPathDistance(futurePosition);

            // are we facing in the correction direction?
            bool rightway = ((pathDistanceOffset > 0) ?
                            (currentPathDistance < futurePathDistance) :
                            (currentPathDistance > futurePathDistance));

            // find the point on the path nearest the predicted future position
            Vector3 tangent;
            float outside;
            Vector3 onPath = path.MapPointToPath(futurePosition, out tangent, out outside);

            // no steering is required if (a) our future position is inside
            // the path tube and (b) we are facing in the correct direction
            if ((outside <= 0) && rightway)
            {
                //We're going at max speed, in the right direction. don't need to do anything
                if (vehicle.Speed >= maxSpeed)
                    return Vector3.Zero;

                //Predict vehicle position and sample multiple times, incresingly far along the path
                var seek = path.MapPointToPath(vehicle.PredictFuturePosition(predictionTime / 3), out tangent, out outside);
                for (int i = 0; i < 3; i++)
                {
                    var s = path.MapPointToPath(seek + tangent * vehicle.Speed / (i + 1), out tangent, out outside);

                    //terminate search if we wander outside the path
                    if (outside > 0)
                        break;
                    seek = s;

                    if (annotation != null)
                        annotation.Circle3D(0.3f, seek, Vector3.UnitX, Colors.Green, 6);
                }

                //Steer towards future path point
                return vehicle.SteerForSeek(seek, maxSpeed, annotation);
            }

            // otherwise we need to steer towards a target point obtained
            // by adding pathDistanceOffset to our current path position
            float targetPathDistance = currentPathDistance + pathDistanceOffset;
            Vector3 target = path.MapPathDistanceToPoint(targetPathDistance);

            if (annotation != null)
                annotation.PathFollowing(futurePosition, onPath, target, outside);

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
        public static Vector3 SteerToAvoidObstacle(this IVehicle vehicle, float minTimeToCollision, IObstacle obstacle, IAnnotationService annotation = null)
        {
            Vector3 avoidance = obstacle.steerToAvoid(vehicle as BaseVehicle, minTimeToCollision);

            // XXX more annotation modularity problems (assumes spherical obstacle)
            if (avoidance != Vector3.Zero && annotation != null)
                annotation.AvoidObstacle(minTimeToCollision * vehicle.Speed);

            return avoidance;
        }

        public static Vector3 SteerToAvoidObstacles(this IVehicle vehicle, float minTimeToCollision, IEnumerable<IObstacle> obstacles, Vector3? referencePoint, IAnnotationService annotation = null)
        {
            var avoidance = Obstacle.steerToAvoidObstacles(vehicle as BaseVehicle, 
                                                           minTimeToCollision,
                                                           obstacles,
                                                           out var nearest,
                                                           referencePoint);

            // XXX more annotation modularity problems (assumes spherical obstacle)
            if (annotation != null && avoidance != Vector3.Zero)
                annotation.AvoidObstacle(minTimeToCollision * vehicle.Speed, nearest);

            return avoidance;
        }

        public static Vector3 SteerForSeparation(this IVehicle vehicle, float maxDistance, float cosMaxAngle, IEnumerable<IVehicle> others, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            Vector3 steering = Vector3.Zero;
            int neighbors = 0;

            // for each of the other vehicles...
            foreach (var other in others)
            {
                if (!IsInBoidNeighborhood(vehicle, other, vehicle.Radius * 3, maxDistance, cosMaxAngle))
                    continue;

                // add in steering contribution
                // (opposite of the offset direction, divided once by distance
                // to normalize, divided another time to get 1/d falloff)
                Vector3 offset = other.Position - vehicle.Position;
                float distanceSquared = Vector3.Dot(offset, offset);
                steering += (offset / -distanceSquared);

                // count neighbors
                neighbors++;
            }

            // divide by neighbors, then normalize to pure direction
            if (neighbors > 0)
            {
                steering = Vector3.Normalize(steering / neighbors);
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
        public static Vector3 SteerToAvoidCloseNeighbors<TVehicle>(this IVehicle vehicle, float minSeparationDistance, IEnumerable<TVehicle> others, IAnnotationService annotation = null)
            where TVehicle : IVehicle
        {
            // for each of the other vehicles...
            foreach (IVehicle other in others)
            {
                if (other != vehicle)
                {
                    float sumOfRadii = vehicle.Radius + other.Radius;
                    float minCenterToCenter = minSeparationDistance + sumOfRadii;
                    Vector3 offset = other.Position - vehicle.Position;
                    float currentDistance = offset.Length();

                    if (currentDistance < minCenterToCenter)
                    {
                        if (annotation != null)
                            annotation.AvoidCloseNeighbor(other, minSeparationDistance);

                        return Vector3Helpers.PerpendicularComponent(-offset, vehicle.Forward);
                    }
                }
            }

            // otherwise return zero
            return Vector3.Zero;
        }

        public static Vector3 SteerForAlignment(this IVehicle vehicle, float maxDistance, float cosMaxAngle, IEnumerable<IVehicle> flock, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            Vector3 steering = Vector3.Zero;
            int neighbors = 0;

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

                var length = steering.Length();
                if (length > 0.025f)
                    steering /= length;
            }

            return steering;
        }

        public static Vector3 SteerForCohesion(this IVehicle vehicle, float maxDistance, float cosMaxAngle, IEnumerable<IVehicle> flock, IAnnotationService annotation = null)
        {
            // steering accumulator and count of neighbors, both initially zero
            Vector3 steering = Vector3.Zero;
            int neighbors = 0;

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
                steering = Vector3.Normalize((steering / neighbors) - vehicle.Position);
            }

            return steering;
        }

        private readonly static float[,] _pursuitFactors = new float[3, 3]
        {
            { 2, 2, 0.5f },         //Behind
            { 4, 0.8f, 1 },         //Aside
            { 0.85f, 1.8f, 4 },     //Ahead
        };

        public static Vector3 SteerForPursuit(this IVehicle vehicle, IVehicle quarry, float maxPredictionTime, float maxSpeed, IAnnotationService annotation = null)
        {
            // offset from this to quarry, that distance, unit vector toward quarry
            Vector3 offset = quarry.Position - vehicle.Position;
            float distance = offset.Length();
            Vector3 unitOffset = offset / distance;

            // how parallel are the paths of "this" and the quarry
            // (1 means parallel, 0 is pependicular, -1 is anti-parallel)
            float parallelness = Vector3.Dot(vehicle.Forward, quarry.Forward);

            // how "forward" is the direction to the quarry
            // (1 means dead ahead, 0 is directly to the side, -1 is straight back)
            float forwardness = Vector3.Dot(vehicle.Forward, unitOffset);

            float directTravelTime = distance / Math.Max(0.001f, vehicle.Speed);
            int f = Utilities.IntervalComparison(forwardness, -0.707f, 0.707f);
            int p = Utilities.IntervalComparison(parallelness, -0.707f, 0.707f);

            // Break the pursuit into nine cases, the cross product of the
            // quarry being [ahead, aside, or behind] us and heading
            // [parallel, perpendicular, or anti-parallel] to us.
            float timeFactor = _pursuitFactors[f + 1, p + 1];

            // estimated time until intercept of quarry
            float et = directTravelTime * timeFactor;

            // xxx experiment, if kept, this limit should be an argument
            float etl = (et > maxPredictionTime) ? maxPredictionTime : et;

            // estimated position of quarry at intercept
            Vector3 target = quarry.PredictFuturePosition(etl);

            // annotation
            if (annotation != null)
                annotation.Line(vehicle.Position, target, Colors.DarkGray);

            return SteerForSeek(vehicle, target, maxSpeed, annotation);
        }

        public static Vector3 SteerForEvasion(this IVehicle vehicle, IVehicle menace, float maxPredictionTime, float maxSpeed, IAnnotationService annotation = null)
        {
            // offset from this to menace, that distance, unit vector toward menace
            Vector3 offset = menace.Position - vehicle.Position;
            float distance = offset.Length();

            float roughTime = distance / menace.Speed;
            float predictionTime = ((roughTime > maxPredictionTime) ? maxPredictionTime : roughTime);

            Vector3 target = menace.PredictFuturePosition(predictionTime);

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
        public static Vector3 SteerForTargetSpeed(this IVehicle vehicle, float targetSpeed, float maxForce, IAnnotationService annotation = null)
        {
            float mf = maxForce;
            float speedError = targetSpeed - vehicle.Speed;
            return vehicle.Forward * Utilities.Clamp(speedError, -mf, +mf);
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
        public static Vector3 SteerToAvoidNeighbors(this IVehicle vehicle, float minTimeToCollision, IEnumerable<IVehicle> others, IAnnotationService annotation = null)
        {
            // first priority is to prevent immediate interpenetration
            Vector3 separation = SteerToAvoidCloseNeighbors(vehicle, 0, others, annotation);
            if (separation != Vector3.Zero)
                return separation;

            // otherwise, go on to consider potential future collisions
            float steer = 0;
            IVehicle threat = null;

            // Time (in seconds) until the most immediate collision threat found
            // so far.  Initial value is a threshold: don't look more than this
            // many frames into the future.
            float minTime = minTimeToCollision;

            // xxx solely for annotation
            Vector3 xxxThreatPositionAtNearestApproach = Vector3.Zero;
            Vector3 xxxOurPositionAtNearestApproach = Vector3.Zero;

            // for each of the other vehicles, determine which (if any)
            // pose the most immediate threat of collision.
            foreach (IVehicle other in others)
            {
                if (other != vehicle)
                {
                    // avoid when future positions are this close (or less)
                    float collisionDangerThreshold = vehicle.Radius * 2;

                    // predicted time until nearest approach of "this" and "other"
                    float time = PredictNearestApproachTime(vehicle, other);

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
                float parallelness = Vector3.Dot(vehicle.Forward, threat.Forward);
                const float ANGLE = 0.707f;

                if (parallelness < -ANGLE)
                {
                    // anti-parallel "head on" paths:
                    // steer away from future threat position
                    Vector3 offset = xxxThreatPositionAtNearestApproach - vehicle.Position;
                    float sideDot = Vector3.Dot(offset, vehicle.Side);
                    steer = (sideDot > 0) ? -1.0f : 1.0f;
                }
                else
                {
                    if (parallelness > ANGLE)
                    {
                        // parallel paths: steer away from threat
                        Vector3 offset = threat.Position - vehicle.Position;
                        float sideDot = Vector3.Dot(offset, vehicle.Side);
                        steer = (sideDot > 0) ? -1.0f : 1.0f;
                    }
                    else
                    {
                        // perpendicular paths: steer behind threat
                        // (only the slower of the two does this)
                        if (threat.Speed <= vehicle.Speed)
                        {
                            float sideDot = Vector3.Dot(vehicle.Side, threat.Velocity);
                            steer = (sideDot > 0) ? -1.0f : 1.0f;
                        }
                    }
                }

                if (annotation != null)
                    annotation.AvoidNeighbor(threat, steer, xxxOurPositionAtNearestApproach, xxxThreatPositionAtNearestApproach);
            }

            return vehicle.Side * steer;
        }

        public static Vector3 SteerToAvoidNeighbors2(this IVehicle vehicle, float minTimeToCollision, IEnumerable<IVehicle> others, IAnnotationService annotation = null)
        {
            // https://www.jdxdev.com/blog/2021/03/19/boids-for-rts/
            // https://howtorts.github.io/2014/01/14/avoidance-behaviours.html
            // 

            System.Func<IVehicle, Vector3> avoidThreatEntity = threat =>
            {
                Vector3 resultVector = Vector3.Zero;
                var obstacle = new SphericalObstacle(threat.Radius, threat.Position);
                PathIntersection intersection = PathIntersection.DEFAULT;
                obstacle.findIntersectionWithVehiclePath(vehicle as BaseVehicle, ref intersection);
                if (intersection.intersect)
                {
                    // Choose right direction
                    resultVector = new Vector3(intersection.surfaceNormal.Z, 0.0f, -intersection.surfaceNormal.X);
                    resultVector.Normalize();

                    //Move it out based on our radius + theirs
                    resultVector *= vehicle.Radius + threat.Radius;

                    if (null != annotation) annotation.AvoidNeighbor2(threat, intersection);
                }
                return resultVector;
            };

            // Find min distance entity
            const float minSeparationDistance = 0.0f;
            foreach (var other in others)
            {
                if (other != vehicle)
                {
                    float sumOfRadii = vehicle.Radius + other.Radius;
                    float minCenterToCenter = minSeparationDistance + sumOfRadii;
                    Vector3 offset = other.Position - vehicle.Position;
                    float currentDistance = offset.Length();

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
            float minTime = minTimeToCollision;

            // xxx solely for annotation
            Vector3 xxxThreatPositionAtNearestApproach = Vector3.Zero;
            Vector3 xxxOurPositionAtNearestApproach = Vector3.Zero;

            // for each of the other vehicles, determine which (if any)
            // pose the most immediate threat of collision.
            foreach (var other in others)
            {
                if (other != vehicle)
                {
                    // avoid when future positions are this close (or less)
                    float collisionDangerThreshold = vehicle.Radius * 2;

                    // predicted time until nearest approach of "this" and "other"
                    float time = PredictNearestApproachTime(vehicle, other);

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
                                = hisPositionAtNearestApproach?? Vector3.Zero;
                            xxxOurPositionAtNearestApproach
                                = ourPositionAtNearestApproach?? Vector3.Zero;
                        }
                    }
                }
            }

            // if a potential collision was found, compute steering to avoid
            if (threat != null)
            {
                return avoidThreatEntity(threat);
            }

            // otherwise return zero
            return Vector3.Zero;
        }

        /// <summary>
        /// Given two vehicles, based on their current positions and velocities,
        /// determine the time until nearest approach
        /// </summary>
        /// <param name="vehicle"></param>
        /// <param name="other"></param>
        /// <returns></returns>
        private static float PredictNearestApproachTime(IVehicle vehicle, IVehicle other)
        {
            // imagine we are at the origin with no velocity,
            // compute the relative velocity of the other this
            Vector3 myVelocity = vehicle.Velocity;
            Vector3 otherVelocity = other.Velocity;
            Vector3 relVelocity = otherVelocity - myVelocity;
            float relSpeed = relVelocity.Length();

            // for parallel paths, the vehicles will always be at the same distance,
            // so return 0 (aka "now") since "there is no time like the present"
            if (Math.Abs(relSpeed - 0) < float.Epsilon)
                return 0;

            // Now consider the path of the other this in this relative
            // space, a line defined by the relative position and velocity.
            // The distance from the origin (our this) to that line is
            // the nearest approach.

            // Take the unit tangent along the other this's path
            Vector3 relTangent = relVelocity / relSpeed;

            // find distance from its path to origin (compute offset from
            // other to us, find length of projection onto path)
            Vector3 relPosition = vehicle.Position - other.Position;
            float projection = Vector3.Dot(relTangent, relPosition);

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
        private static float ComputeNearestApproachPositions(IVehicle vehicle, IVehicle other, float time, 
            out Vector3? ourPositionAtNearestApproach, out Vector3? hisPositionAtNearestApproach)
        {
            Vector3 myTravel = vehicle.Forward * vehicle.Speed * time;
            Vector3 otherTravel = other.Forward * other.Speed * time;

            Vector3 myFinal = vehicle.Position + myTravel;
            Vector3 otherFinal = other.Position + otherTravel;

            // xxx for annotation
            ourPositionAtNearestApproach = myFinal;
            hisPositionAtNearestApproach = otherFinal;

            return Vector3.Distance(myFinal, otherFinal);
        }

        public static bool IsAhead(this IVehicle vehicle, Vector3 target, float cosThreshold = 0.707f)
        {
            Vector3 targetDirection = Vector3.Normalize(target - vehicle.Position);
            return Vector3.Dot(vehicle.Forward, targetDirection) > cosThreshold;
        }

        public static bool IsAside(this IVehicle vehicle, Vector3 target, float cosThreshold = 0.707f)
        {
            Vector3 targetDirection = Vector3.Normalize(target - vehicle.Position);
            float dp = Vector3.Dot(vehicle.Forward, targetDirection);
            return (dp < cosThreshold) && (dp > -cosThreshold);
        }

        public static bool IsBehind(this IVehicle vehicle, Vector3 target, float cosThreshold = -0.707f)
        {
            Vector3 targetDirection = Vector3.Normalize(target - vehicle.Position);
            return Vector3.Dot(vehicle.Forward, targetDirection) < cosThreshold;
        }

        private static bool IsInBoidNeighborhood(this ILocalSpaceBasis vehicle, ILocalSpaceBasis other, float minDistance, float maxDistance, float cosMaxAngle)
        {
            if (other == vehicle)
                return false;

            Vector3 offset = other.Position - vehicle.Position;
            float distanceSquared = offset.LengthSquared();

            // definitely in neighborhood if inside minDistance sphere
            if (distanceSquared < (minDistance * minDistance))
                return true;

            // definitely not in neighborhood if outside maxDistance sphere
            if (distanceSquared > (maxDistance * maxDistance))
                return false;

            // otherwise, test angular offset from forward axis
            Vector3 unitOffset = offset / (float)Math.Sqrt(distanceSquared);
            float forwardness = Vector3.Dot(vehicle.Forward, unitOffset);
            return forwardness > cosMaxAngle;
        }
    }
}
