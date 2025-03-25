
using FixMath;
using SharpSteer2;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds.SteeringForce
{
    public class FollowPathForce : AbstractSteeringForce
    {
        // Path following look-ahead time
        public FixMath.F64 FollowPathAheadTime = FixMath.F64.FromDouble(3.0f);

        // True means walking forward along the path, false means walking backward along the path
        public bool PathDirection = true;

        public override F64Vec3 GetSteeringForce(MovableEntity owner)
        {
            var annotation = owner.Annotation;
            var pathway = owner.Pathway;
            var maxSpeed = owner.MaxSpeed;
            var maxForce = owner.MaxForce;
            var predictionTime = FollowPathAheadTime;
            var pathDirection = PathDirection;

            var pathFollowForce = FixMath.F64Vec3.Zero;
            {
                if (null != pathway)
                {
                    pathFollowForce = owner.SteerToFollowPath(pathDirection, predictionTime, pathway, maxSpeed, out var currentPathDistance, annotation) * Weight;
                }

                if (pathFollowForce == FixMath.F64Vec3.Zero)
                {
                    pathFollowForce = owner.SteerForSeek(owner.TargetLocation.Value, maxSpeed) * Weight;
                }

                pathFollowForce = pathFollowForce.TruncateLength(maxForce);
            }
            return pathFollowForce;
        }

        public override void Reset()
        {
            
        }
    }
}
