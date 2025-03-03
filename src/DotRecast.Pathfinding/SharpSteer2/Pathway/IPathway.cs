using System.Numerics;

namespace SharpSteer2.Pathway
{
    public interface IPathway
    {
        /// <summary>
        /// Given an arbitrary point ("A"), returns the nearest point ("P") on
        /// this path.  Also returns, via output arguments, the path tangent at
        /// P and a measure of how far A is outside the Pathway's "tube".  Note
        /// that a negative distance indicates A is inside the Pathway.
        /// </summary>
        /// <param name="point"></param>
        /// <param name="tangent"></param>
        /// <param name="outside"></param>
        /// <returns></returns>
        FixMath.F64Vec3 MapPointToPath(FixMath.F64Vec3 point, out FixMath.F64Vec3 tangent, out FixMath.F64 outside);

        /// <summary>
        /// given a distance along the path, convert it to a point on the path
        /// </summary>
        /// <param name="pathDistance"></param>
        /// <returns></returns>
        FixMath.F64Vec3 MapPathDistanceToPoint(FixMath.F64 pathDistance);

        /// <summary>
        /// Given an arbitrary point, convert it to a distance along the path.
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        FixMath.F64 MapPointToPathDistance(FixMath.F64Vec3 point);
    }
}
