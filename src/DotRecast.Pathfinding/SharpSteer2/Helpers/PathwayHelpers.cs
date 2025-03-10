﻿using System.Numerics;
using SharpSteer2.Pathway;

namespace SharpSteer2.Helpers
{
    public static class PathwayHelpers
    {
        /// <summary>
        /// is the given point inside the path tube?
        /// </summary>
        /// <param name="pathway"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static bool IsInsidePath(this IPathway pathway, FixMath.F64Vec3 point)
		{
			FixMath.F64 outside;
			FixMath.F64Vec3 tangent;
            pathway.MapPointToPath(point, out tangent, out outside);
			return outside < 0;
		}

        /// <summary>
        /// how far outside path tube is the given point?  (negative is inside)
        /// </summary>
        /// <param name="pathway"></param>
        /// <param name="point"></param>
        /// <returns></returns>
        public static FixMath.F64 HowFarOutsidePath(this IPathway pathway, FixMath.F64Vec3 point)
		{
			FixMath.F64 outside;
			FixMath.F64Vec3 tangent;
            pathway.MapPointToPath(point, out tangent, out outside);
			return outside;
		}
    }
}
