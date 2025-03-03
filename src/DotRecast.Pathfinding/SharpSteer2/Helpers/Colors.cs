using System.Numerics;

namespace SharpSteer2.Helpers
{
    internal static class Colors
    {
        public static readonly FixMath.F64Vec3 White = FixMath.F64Vec3.FromFloat(1, 1, 1);

        public static readonly FixMath.F64Vec3 Orange = FixMath.F64Vec3.FromFloat(1, 165f / 255f, 0);

        public static readonly FixMath.F64Vec3 OrangeRed = FixMath.F64Vec3.FromFloat(1, 69f / 255f, 9);

        public static readonly FixMath.F64Vec3 Gold = FixMath.F64Vec3.FromFloat(1, 215f / 255f, 0);

        public static readonly FixMath.F64Vec3 Red = FixMath.F64Vec3.FromFloat(1, 0, 0);

        public static readonly FixMath.F64Vec3 Green = FixMath.F64Vec3.FromFloat(0, 1, 0);

        public static readonly FixMath.F64Vec3 DarkGray = FixMath.F64Vec3.FromFloat(87f / 255f, 87f / 255f, 87f / 255f);
    }
}
