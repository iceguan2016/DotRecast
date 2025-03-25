
using FixMath;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class FlockSeparationForce : AbstractSteeringForce
    {
        public FixMath.F64 SeparationRadius = FixMath.F64.FromFloat(3.0f);
        public FixMath.F64 SeparationAngle = FixMath.F64.FromFloat(-0.707f);

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var neighbors = owner.Neighbors;

            // steering accumulator and count of neighbors, both initially zero
            var steering = FixMath.F64Vec3.Zero;
            var neighborCount = 0;

            // for each of the other vehicles...
            foreach (var other in neighbors)
            {
                if (!VehicleHelpers.IsInBoidNeighborhood(owner, other, owner.Radius * 3, SeparationRadius, SeparationAngle))
                    continue;

                // add in steering contribution
                // (opposite of the offset direction, divided once by distance
                // to normalize, divided another time to get 1/d falloff)
                var offset = other.Position - owner.Position;
                var distanceSquared = FixMath.F64Vec3.Dot(offset, offset);
                steering += (offset / -distanceSquared);

                // count neighbors
                neighborCount++;
            }

            // divide by neighbors, then normalize to pure direction
            if (neighborCount > 0)
            {
                steering = FixMath.F64Vec3.NormalizeFast(steering / FixMath.F64.FromInt(neighborCount));
            }

            return steering * Weight;
        }

        public override void Reset()
        {
        }
    }
}
