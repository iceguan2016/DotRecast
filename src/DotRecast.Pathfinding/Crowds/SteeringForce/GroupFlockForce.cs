
using FixMath;

namespace Pathfinding.Crowds.SteeringForce
{
    public class GroupFlockForce : AbstractSteeringForce
    {
        public FixMath.F64 SeparationRadius = FixMath.F64.FromFloat(3.0f);
        public FixMath.F64 SeparationAngle = FixMath.F64.FromFloat(-0.707f);
        public FixMath.F64 SeparationWeight = FixMath.F64.FromFloat(0.0f);

        public FixMath.F64 AlignmentRadius = FixMath.F64.FromFloat(5.0f);
        public FixMath.F64 AlignmentAngle = FixMath.F64.FromFloat(0.707f);
        public FixMath.F64 AlignmentWeight = FixMath.F64.FromFloat(0.0f);

        public FixMath.F64 CohesionRadius = FixMath.F64.FromFloat(4.0f);
        public FixMath.F64 CohesionAngle = FixMath.F64.FromFloat(-0.15f);
        public FixMath.F64 CohesionWeight = FixMath.F64.FromFloat(0.0f);

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var neighbors = owner.Neighbors;
            // determine each of the three component behaviors of flocking
            var separation = owner.SteerForSeparation(SeparationRadius,
                                                SeparationAngle,
                                                neighbors);
            var alignment = owner.SteerForAlignment(AlignmentRadius,
                                              AlignmentAngle,
                                              neighbors);
            var cohesion = owner.SteerForCohesion(CohesionRadius,
                                            CohesionAngle,
                                            neighbors);

            // apply weights to components (save in variables for annotation)
            var separationForce = separation * SeparationWeight;
            var alignmentForce = alignment * AlignmentWeight;
            var cohesionForce = cohesion * CohesionWeight;
            
            return separationForce + alignmentForce + cohesionForce;
        }

        public override void Reset()
        {
        }
    }
}
