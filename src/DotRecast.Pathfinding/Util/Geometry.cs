
namespace Pathfinding.Util
{
    public static class Geometry
    {
        /* Check is c0 will collision with c1 */
        public static bool TimeToCollisionWithCircle2D(
                FixMath.F64Vec2 c0,
                FixMath.F64 r0,
                FixMath.F64Vec2 relativeVel,
                FixMath.F64Vec2 c1,
                FixMath.F64 r1,
                out FixMath.F64 tmin, /*out param*/
                out FixMath.F64 tmax /*out param*/)
        {
            // P = tV and (P - (c1-c0)*(c1-c0))^2 = (r1+r0)^2 intersect point
            tmin = FixMath.F64.Zero;
            tmax = FixMath.F64.Zero;
            var s = c1 - c0;
            var r = r0 + r1;
            var c = FixMath.F64Vec2.LengthSqr(s) - r * r;
            var a = FixMath.F64Vec2.LengthSqr(relativeVel);
            if (a < FixMath.F64.Epsilon)
                return false;        // not moving

            // Overlap, calculate time to exit.
            var b = FixMath.F64Vec2.Dot(relativeVel, s);
            var d = b * b - a * c;
            if (d < FixMath.F64.Zero)
                return false; // no intersection.
            a = FixMath.F64.One / a;
            var rd = FixMath.F64.Sqrt(d);
            tmin = (b - rd) * a;
            tmax = (b + rd) * a;
            return true;
        }
    }
}
