using System.Numerics;
using System.Runtime.CompilerServices;

namespace SharpSteer2.Helpers
{
    public static class MatrixHelpers
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Right(ref FixMath.F64Matrix m, ref FixMath.F64Vec3 v)
        {
            m.SetScaledAxis(FixMath.F64Matrix.eAxis.X, v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixMath.F64Vec3 Right(ref FixMath.F64Matrix m)
        {
            return m.GetScaledAxis(FixMath.F64Matrix.eAxis.X);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Up(ref FixMath.F64Matrix m, ref FixMath.F64Vec3 v)
        {
            m.SetScaledAxis(FixMath.F64Matrix.eAxis.Y, v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixMath.F64Vec3 Up(ref FixMath.F64Matrix m)
        {
            return m.GetScaledAxis(FixMath.F64Matrix.eAxis.Y);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Backward(ref FixMath.F64Matrix m, ref FixMath.F64Vec3 v)
        {
            m.SetScaledAxis(FixMath.F64Matrix.eAxis.Z, v);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FixMath.F64Vec3 Backward(ref FixMath.F64Matrix m)
        {
            return m.GetScaledAxis(FixMath.F64Matrix.eAxis.Z);
        }
    }
}
