
using FixMath;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class FlockCohesionForce : AbstractSteeringForce
    {
        public FixMath.F64 CohesionRadius = FixMath.F64.FromFloat(4.0f);
        public FixMath.F64 CohesionAngle = FixMath.F64.FromFloat(-0.15f);

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var neighbors = owner.Neighbors;

            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighborCount = 0;

            // for each of the other vehicles...
            foreach (var other in neighbors)
            {
                if (!VehicleHelpers.IsInBoidNeighborhood(owner, other, owner.Radius * 3, CohesionRadius, CohesionAngle))
                    continue;

                // accumulate sum of neighbor's positions
                steering += other.Position;

                // count neighbors
                neighborCount++;
            }

            // divide by neighbors, subtract off current position to get error-
            // correcting direction, then normalize to pure direction
            if (neighborCount > 0)
            {
                steering = FixMath.F64Vec3.NormalizeFast((steering / FixMath.F64.FromInt(neighborCount)) - owner.Position);
            }

            return steering * Weight;
        }

        public override void Reset()
        {
        }
    }
}
