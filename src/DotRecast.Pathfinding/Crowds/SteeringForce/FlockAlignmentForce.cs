
using FixMath;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class FlockAlignmentForce : AbstractSteeringForce
    {
        public FixMath.F64 AlignmentRadius = FixMath.F64.FromFloat(5.0f);
        public FixMath.F64 AlignmentAngle = FixMath.F64.FromFloat(0.707f);

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var neighbors = owner.Neighbors;

            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighborCount = 0;

            // for each of the other vehicles...
            foreach (var other in neighbors)
            {
                // moving entity only
                if (other is MovableEntity entity)
                {
                    if (!entity.HasEntityState(MovableEntity.eEntityState.Moving))
                    {
                        continue;
                    }
                }

                if (!VehicleHelpers.IsInBoidNeighborhood(owner, other, owner.Radius * 3, AlignmentRadius, AlignmentAngle))
                    continue;
                // accumulate sum of neighbor's heading
                steering += other.Forward;

                // count neighbors
                neighborCount++;
            }

            // divide by neighbors, subtract off current heading to get error-
            // correcting direction, then normalize to pure direction
            if (neighborCount > 0)
            {
                steering = ((steering / FixMath.F64.FromInt(neighborCount)) - owner.Forward);

                var length = FixMath.F64Vec3.LengthFast(steering);
                if (length > FixMath.F64.FromFloat(0.025f))
                    steering /= length;
            }

            return steering * Weight;
        }

        public override void Reset()
        {
        }
    }
}
