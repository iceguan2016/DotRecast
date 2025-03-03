using System.Numerics;

namespace SharpSteer2
{
    /// <summary>
    /// A flow field which can be sampled at arbitrary locations
    /// </summary>
    public interface IFlowField
    {
        /// <summary>
        /// Sample the flow field at the given location
        /// </summary>
        /// <param name="location"></param>
        /// <returns></returns>
        FixMath.F64Vec3 Sample(FixMath.F64Vec3 location);
    }
}
