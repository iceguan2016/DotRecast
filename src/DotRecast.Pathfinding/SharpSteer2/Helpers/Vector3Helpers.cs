// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

namespace SharpSteer2.Helpers
{
    public static class Vector3Helpers
    {
        /// <summary>
        /// return component of vector parallel to a unit basis vector
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="unitBasis">A unit length basis vector</param>
        /// <returns></returns>
        public static FixMath.F64Vec3 ParallelComponent(FixMath.F64Vec3 vector, FixMath.F64Vec3 unitBasis)
        {
            var projection = FixMath.F64Vec3.Dot(vector, unitBasis);
            return unitBasis * projection;
        }

        /// <summary>
        /// return component of vector perpendicular to a unit basis vector
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="unitBasis">A unit length basis vector</param>
        /// <returns></returns>
        public static FixMath.F64Vec3 PerpendicularComponent(FixMath.F64Vec3 vector, FixMath.F64Vec3 unitBasis)
        {
            return (vector - ParallelComponent(vector, unitBasis));
        }

        /// <summary>
        /// clamps the length of a given vector to maxLength.  If the vector is
        /// shorter its value is returned unaltered, if the vector is longer
        /// the value returned has length of maxLength and is parallel to the
        /// original input.
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="maxLength"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 TruncateLength(this FixMath.F64Vec3 vector, FixMath.F64 maxLength)
        {
            var maxLengthSquared = maxLength * maxLength;
            var vecLengthSquared = FixMath.F64Vec3.LengthSqr(vector);
            if (vecLengthSquared <= maxLengthSquared)
                return vector;

            return (vector * (maxLength / FixMath.F64.SqrtFast(vecLengthSquared)));
        }

        /// <summary>
        /// rotate this vector about the global Y (up) axis by the given angle
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="radians"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 RotateAboutGlobalY(this FixMath.F64Vec3 vector, FixMath.F64 radians)
        {
            var s = FixMath.F64.Zero;
            var c = FixMath.F64.Zero;
            return RotateAboutGlobalY(vector, radians, ref s, ref c);
        }

        /// <summary>
        /// Rotate this vector about the global Y (up) axis by the given angle
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="radians"></param>
        /// <param name="sin">Either Sin(radians) or default(float), if default(float) this value will be initialized with Sin(radians)</param>
        /// <param name="cos">Either Cos(radians) or default(float), if default(float) this value will be initialized with Cos(radians)</param>
        /// <returns></returns>
        public static FixMath.F64Vec3 RotateAboutGlobalY(this FixMath.F64Vec3 vector, FixMath.F64 radians, ref FixMath.F64 sin, ref FixMath.F64 cos)
        {
            // if both are default, they have not been initialized yet
// ReSharper disable CompareOfFloatsByEqualityOperator
            if (sin == FixMath.F64.Zero && cos == FixMath.F64.Zero)
// ReSharper restore CompareOfFloatsByEqualityOperator
            {
                sin = FixMath.F64.Sin(radians);
                cos = FixMath.F64.Cos(radians);
            }
            return new FixMath.F64Vec3((vector.X * cos) + (vector.Z * sin), vector.Y, (vector.Z * cos) - (vector.X * sin));
        }

        /// <summary>
        /// Wrap a position around so it is always within 1 radius of the sphere (keeps repeating wrapping until position is within sphere)
        /// </summary>
        /// <param name="vector"></param>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 SphericalWrapAround(this FixMath.F64Vec3 vector, FixMath.F64Vec3 center, FixMath.F64 radius)
        {
            var r = FixMath.F64.Zero;
            do
            {
                var offset = vector - center;
                r = FixMath.F64Vec3.LengthFast(offset);

                if (r > radius)
                    vector = vector + ((offset / r) * radius * (-FixMath.F64.Two));

            } while (r > radius);

            return vector;
        }

        /// <summary>
        /// Returns a position randomly distributed on a disk of unit radius
        /// on the XZ (Y=0) plane, centered at the origin.  Orientation will be
        /// random and length will range between 0 and 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64Vec3 RandomVectorOnUnitRadiusXZDisk()
        {
            var v = FixMath.F64Vec3.Zero;
            do
            {
                v.X = (RandomHelpers.Random() * 2) - 1;
                v.Y = FixMath.F64.Zero;
                v.Z = (RandomHelpers.Random() * 2) - 1;
            }
            while (FixMath.F64Vec3.LengthFast(v) >= 1);

            return v;
        }

        /// <summary>
        /// Returns a position randomly distributed inside a sphere of unit radius
        /// centered at the origin.  Orientation will be random and length will range
        /// between 0 and 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64Vec3 RandomVectorInUnitRadiusSphere()
        {
            var v = FixMath.F64Vec3.Zero;
            do
            {
                v.X = (RandomHelpers.Random() * 2) - 1;
                v.Y = (RandomHelpers.Random() * 2) - 1;
                v.Z = (RandomHelpers.Random() * 2) - 1;
            }
            while (FixMath.F64Vec3.LengthFast(v) >= 1);

            return v;
        }

        /// <summary>
        /// Returns a position randomly distributed on the surface of a sphere
        /// of unit radius centered at the origin.  Orientation will be random
        /// and length will be 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64Vec3 RandomUnitVector()
        {
            return FixMath.F64Vec3.Normalize(RandomVectorInUnitRadiusSphere());
        }

        /// <summary>
        /// Returns a position randomly distributed on a circle of unit radius
        /// on the XZ (Y=0) plane, centered at the origin.  Orientation will be
        /// random and length will be 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64Vec3 RandomUnitVectorOnXZPlane()
        {
            var temp = RandomVectorInUnitRadiusSphere();
            temp.Y = FixMath.F64.Zero;
            temp = FixMath.F64Vec3.NormalizeFast(temp);

            return temp;
        }

        /// <summary>
        /// Clip a vector to be within the given cone
        /// </summary>
        /// <param name="source">A vector to clip</param>
        /// <param name="cosineOfConeAngle">The cosine of the cone angle</param>
        /// <param name="basis">The vector along the middle of the cone</param>
        /// <returns></returns>
        public static FixMath.F64Vec3 LimitMaxDeviationAngle(this FixMath.F64Vec3 source, FixMath.F64 cosineOfConeAngle, FixMath.F64Vec3 basis)
        {
            return LimitDeviationAngleUtility(true, // force source INSIDE cone
                source, cosineOfConeAngle, basis);
        }

        /// <summary>
        /// Clip a vector to be outside the given cone
        /// </summary>
        /// <param name="source">A vector to clip</param>
        /// <param name="cosineOfConeAngle">The cosine of the cone angle</param>
        /// <param name="basis">The vector along the middle of the cone</param>
        /// <returns></returns>
        public static FixMath.F64Vec3 LimitMinDeviationAngle(this FixMath.F64Vec3 source, FixMath.F64 cosineOfConeAngle, FixMath.F64Vec3 basis)
        {
            return LimitDeviationAngleUtility(false, // force source OUTSIDE cone
                source, cosineOfConeAngle, basis);
        }

        /// <summary>
        /// used by limitMaxDeviationAngle / limitMinDeviationAngle
        /// </summary>
        /// <param name="insideOrOutside"></param>
        /// <param name="source"></param>
        /// <param name="cosineOfConeAngle"></param>
        /// <param name="basis"></param>
        /// <returns></returns>
        private static FixMath.F64Vec3 LimitDeviationAngleUtility(bool insideOrOutside, FixMath.F64Vec3 source, FixMath.F64 cosineOfConeAngle, FixMath.F64Vec3 basis)
        {
            // immediately return zero length input vectors
            var sourceLength = FixMath.F64Vec3.LengthFast(source);
            if (sourceLength < FixMath.F64.Epsilon)
                return source;

            // measure the angular diviation of "source" from "basis"
            var direction = source / sourceLength;

            var cosineOfSourceAngle = FixMath.F64Vec3.Dot(direction, basis);

            // Simply return "source" if it already meets the angle criteria.
            // (note: we hope this top "if" gets compiled out since the flag
            // is a constant when the function is inlined into its caller)
            if (insideOrOutside)
            {
                // source vector is already inside the cone, just return it
                if (cosineOfSourceAngle >= cosineOfConeAngle)
                    return source;
            }
            else if (cosineOfSourceAngle <= cosineOfConeAngle)
                return source;

            // find the portion of "source" that is perpendicular to "basis"
            var perp = PerpendicularComponent(source, basis);
            if (perp == FixMath.F64Vec3.Zero)
                return FixMath.F64Vec3.Zero;

            // normalize that perpendicular
            var unitPerp = FixMath.F64Vec3.NormalizeFast(perp);

            // construct a new vector whose length equals the source vector,
            // and lies on the intersection of a plane (formed the source and
            // basis vectors) and a cone (whose axis is "basis" and whose
            // angle corresponds to cosineOfConeAngle)
            var perpDist = FixMath.F64.Sqrt(1 - (cosineOfConeAngle * cosineOfConeAngle));
            var c0 = basis * cosineOfConeAngle;
            var c1 = unitPerp * perpDist;
            return (c0 + c1) * sourceLength;
        }

        /// <summary>
        /// Returns the distance between a point and a line.
        /// </summary>
        /// <param name="point">The point to measure distance to</param>
        /// <param name="lineOrigin">A point on the line</param>
        /// <param name="lineUnitTangent">A UNIT vector parallel to the line</param>
        /// <returns></returns>
        public static FixMath.F64 DistanceFromLine(this FixMath.F64Vec3 point, FixMath.F64Vec3 lineOrigin, FixMath.F64Vec3 lineUnitTangent)
        {
            var offset = point - lineOrigin;
            var perp = PerpendicularComponent(offset, lineUnitTangent);
            return FixMath.F64Vec3.LengthFast(perp);
        }

        /// <summary>
        /// Find any arbitrary vector which is perpendicular to the given vector
        /// </summary>
        /// <param name="direction"></param>
        /// <returns></returns>
        public static FixMath.F64Vec3 FindPerpendicularIn3d(this FixMath.F64Vec3 direction)
        {
            // to be filled in:
            FixMath.F64Vec3 quasiPerp;  // a direction which is "almost perpendicular"
            FixMath.F64Vec3 result;     // the computed perpendicular to be returned

            // three mutually perpendicular basis vectors
            FixMath.F64Vec3 i = FixMath.F64Vec3.AxisX;
            FixMath.F64Vec3 j = FixMath.F64Vec3.AxisY;
            FixMath.F64Vec3 k = FixMath.F64Vec3.AxisZ;

            // measure the projection of "direction" onto each of the axes
            var id = FixMath.F64Vec3.Dot(i, direction);
            var jd = FixMath.F64Vec3.Dot(j, direction);
            var kd = FixMath.F64Vec3.Dot(k, direction);

            // set quasiPerp to the basis which is least parallel to "direction"
            if ((id <= jd) && (id <= kd))
                quasiPerp = i;           // projection onto i was the smallest
            else if ((jd <= id) && (jd <= kd))
                quasiPerp = j;           // projection onto j was the smallest
            else
                quasiPerp = k;           // projection onto k was the smallest

            // return the cross product (direction x quasiPerp)
            // which is guaranteed to be perpendicular to both of them
            result = FixMath.F64Vec3.Cross(direction, quasiPerp);

            return result;
        }

        public static FixMath.F64Vec3 Forward { get { return FixMath.F64Vec3.AxisZ; } }
        public static FixMath.F64Vec3 Right { get { return FixMath.F64Vec3.AxisX; } }
        public static FixMath.F64Vec3 Up { get { return FixMath.F64Vec3.AxisY; } }

        public static FixMath.F64 Dot(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { return FixMath.F64Vec3.Dot(a, b); }
        public static FixMath.F64Vec3 Cross(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { return FixMath.F64Vec3.Cross(a, b); }

        public static FixMath.F64Vec3 Normalize(this FixMath.F64Vec3 a) { return FixMath.F64Vec3.NormalizeFast(a); }
        public static FixMath.F64 Length(this FixMath.F64Vec3 a) { return FixMath.F64Vec3.LengthFast(a); }
        public static FixMath.F64 Length2D(this FixMath.F64Vec3 a) { var b = new FixMath.F64Vec3(a.X, FixMath.F64.Zero, a.Z); return FixMath.F64Vec3.LengthFast(b); }
        public static FixMath.F64 Distance(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { return FixMath.F64Vec3.DistanceFast(a, b); }
        public static FixMath.F64 Distance2D(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { var c = a - b; c.Y = FixMath.F64.Zero; return FixMath.F64Vec3.LengthFast(c); }
        public static FixMath.F64 DistanceSquared(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { return FixMath.F64Vec3.LengthSqr(a - b); }
        public static FixMath.F64 DistanceSquared2D(this FixMath.F64Vec3 a, FixMath.F64Vec3 b) { var c = a - b; c.Y = FixMath.F64.Zero; return FixMath.F64Vec3.LengthSqr(c); }
        public static FixMath.F64Vec3 SetYtoZero(this FixMath.F64Vec3 a) { return new FixMath.F64Vec3(a.X, FixMath.F64.Zero, a.Z); }
    }
}
