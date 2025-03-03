using System.Numerics;

namespace SharpSteer2.Helpers
{
    public static class LocalSpaceBasisHelpers
    {
        /// <summary>
        /// Transforms a direction in global space to its equivalent in local space.
        /// </summary>
        /// <param name="basis">The basis which this should operate on</param>
        /// <param name="globalDirection">The global space direction to transform.</param>
        /// <returns>The global space direction transformed to local space .</returns>
        public static FixMath.F64Vec3 LocalizeDirection(this ILocalSpaceBasis basis, FixMath.F64Vec3 globalDirection)
        {
            // dot offset with local basis vectors to obtain local coordiantes
            return new FixMath.F64Vec3(FixMath.F64Vec3.Dot(globalDirection, basis.Side), FixMath.F64Vec3.Dot(globalDirection, basis.Up), FixMath.F64Vec3.Dot(globalDirection, basis.Forward));
        }

        /// <summary>
        /// Transforms a point in global space to its equivalent in local space.
        /// </summary>
        /// <param name="basis">The basis which this should operate on</param>
        /// <param name="globalPosition">The global space position to transform.</param>
        /// <returns>The global space position transformed to local space.</returns>
        public static FixMath.F64Vec3 LocalizePosition(this ILocalSpaceBasis basis, FixMath.F64Vec3 globalPosition)
        {
            // global offset from local origin
            var globalOffset = globalPosition - basis.Position;

            // dot offset with local basis vectors to obtain local coordiantes
            return LocalizeDirection(basis, globalOffset);
        }

        /// <summary>
        /// Transforms a point in local space to its equivalent in global space.
        /// </summary>
        /// <param name="basis">The basis which this should operate on</param>
        /// <param name="localPosition">The local space position to tranform.</param>
        /// <returns>The local space position transformed to global space.</returns>
        public static FixMath.F64Vec3 GlobalizePosition(this ILocalSpaceBasis basis, FixMath.F64Vec3 localPosition)
        {
            return basis.Position + GlobalizeDirection(basis, localPosition);
        }

        /// <summary>
        /// Transforms a direction in local space to its equivalent in global space.
        /// </summary>
        /// <param name="basis">The basis which this should operate on</param>
        /// <param name="localDirection">The local space direction to tranform.</param>
        /// <returns>The local space direction transformed to global space</returns>
        public static FixMath.F64Vec3 GlobalizeDirection(this ILocalSpaceBasis basis, FixMath.F64Vec3 localDirection)
        {
            return ((basis.Side * localDirection.X) +
                    (basis.Up * localDirection.Y) +
                    (basis.Forward * localDirection.Z));
        }

        /// <summary>
        /// Rotates, in the canonical direction, a vector pointing in the
        /// "forward" (+Z) direction to the "side" (+/-X) direction as implied
        /// by IsRightHanded.
        /// </summary>
        /// <param name="basis">The basis which this should operate on</param>
        /// <param name="value">The local space vector.</param>
        /// <returns>The rotated vector.</returns>
        public static FixMath.F64Vec3 LocalRotateForwardToSide(this ILocalSpaceBasis basis, FixMath.F64Vec3 value)
        {
            return new FixMath.F64Vec3(-value.Z, value.Y, value.X);
        }

        public static void ResetLocalSpace(out FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, out FixMath.F64Vec3 up, out FixMath.F64Vec3 position)
        {
            forward = -FixMath.F64Vec3.AxisZ;
            side = FixMath.F64Vec3.AxisX;
            up = FixMath.F64Vec3.AxisY;
            position = FixMath.F64Vec3.Zero;
        }

        /// <summary>
        /// set "side" basis vector to normalized cross product of forward and up
        /// </summary>
        /// <param name="forward"></param>
        /// <param name="side"></param>
        /// <param name="up"></param>
        public static void SetUnitSideFromForwardAndUp(ref FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, ref FixMath.F64Vec3 up)
        {
            // derive new unit side basis vector from forward and up
            side = FixMath.F64Vec3.NormalizeFast(FixMath.F64Vec3.Cross(forward, up));
        }

        /// <summary>
        /// regenerate the orthonormal basis vectors given a new forward
        /// (which is expected to have unit length)
        /// </summary>
        /// <param name="newUnitForward"></param>
        /// <param name="forward"></param>
        /// <param name="side"></param>
        /// <param name="up"></param>
        public static void RegenerateOrthonormalBasisUF(FixMath.F64Vec3 newUnitForward, out FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, ref FixMath.F64Vec3 up)
        {
            forward = newUnitForward;

            // derive new side basis vector from NEW forward and OLD up
            SetUnitSideFromForwardAndUp(ref forward, out side, ref up);

            // derive new Up basis vector from new Side and new Forward
            //(should have unit length since Side and Forward are
            // perpendicular and unit length)
            up = FixMath.F64Vec3.Cross(side, forward);
        }

        /// <summary>
        /// for when the new forward is NOT know to have unit length
        /// </summary>
        /// <param name="newForward"></param>
        /// <param name="forward"></param>
        /// <param name="side"></param>
        /// <param name="up"></param>
        public static void RegenerateOrthonormalBasis(FixMath.F64Vec3 newForward, out FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, ref FixMath.F64Vec3 up)
        {
            RegenerateOrthonormalBasisUF(FixMath.F64Vec3.NormalizeFast(newForward), out forward, out side, ref up);
        }

        /// <summary>
        /// for supplying both a new forward and and new up
        /// </summary>
        /// <param name="newForward"></param>
        /// <param name="newUp"></param>
        /// <param name="forward"></param>
        /// <param name="side"></param>
        /// <param name="up"></param>
        public static void RegenerateOrthonormalBasis(FixMath.F64Vec3 newForward, FixMath.F64Vec3 newUp, out FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, out FixMath.F64Vec3 up)
        {
            up = newUp;
            RegenerateOrthonormalBasis(FixMath.F64Vec3.NormalizeFast(newForward), out forward, out side, ref up);
        }

        public static FixMath.F64Matrix ToMatrix(this ILocalSpaceBasis basis)
        {
            return ToMatrix(basis.Forward, basis.Side, basis.Up, basis.Position);
        }

        public static FixMath.F64Matrix ToMatrix(FixMath.F64Vec3 forward, FixMath.F64Vec3 side, FixMath.F64Vec3 up, FixMath.F64Vec3 position)
        {
            var m = new FixMath.F64Matrix();
            m.SetIdentity();
            m.SetTranslation(position);
            MatrixHelpers.Right(ref m, ref side);
            MatrixHelpers.Up(ref m, ref up);
            MatrixHelpers.Right(ref m, ref forward);

            return m;
        }

        public static void FromMatrix(FixMath.F64Matrix transformation, out FixMath.F64Vec3 forward, out FixMath.F64Vec3 side, out FixMath.F64Vec3 up, out FixMath.F64Vec3 position)
        {
            position = transformation.GetTranslation();
            side = MatrixHelpers.Right(ref transformation);
            up = MatrixHelpers.Up(ref transformation);
            forward = MatrixHelpers.Backward(ref transformation);
        }
    }
}
