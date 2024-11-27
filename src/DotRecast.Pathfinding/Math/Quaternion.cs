using System;
using System.Runtime.CompilerServices;

namespace UnityEngine
{
    /// <summary>
    /// A Quaternion type for representing rotations.
    /// </summary>
    [Serializable]
    public partial struct Quaternion : System.IEquatable<Quaternion>, IFormattable
    {
        /// <summary>Extrinsic rotation order. Specifies in which order rotations around the principal axes (x, y and z) are to be applied.</summary>
        public enum RotationOrder : byte
        {
            /// <summary>Extrinsic rotation around the x axis, then around the y axis and finally around the z axis.</summary>
            XYZ,
            /// <summary>Extrinsic rotation around the x axis, then around the z axis and finally around the y axis.</summary>
            XZY,
            /// <summary>Extrinsic rotation around the y axis, then around the x axis and finally around the z axis.</summary>
            YXZ,
            /// <summary>Extrinsic rotation around the y axis, then around the z axis and finally around the x axis.</summary>
            YZX,
            /// <summary>Extrinsic rotation around the z axis, then around the x axis and finally around the y axis.</summary>
            ZXY,
            /// <summary>Extrinsic rotation around the z axis, then around the y axis and finally around the x axis.</summary>
            ZYX,
            /// <summary>Unity default rotation order. Extrinsic Rotation around the z axis, then around the x axis and finally around the y axis.</summary>
            Default = ZXY
        };

        /// <summary>
        /// Specifies the x-value of the vector component of the Quaternion.
        /// </summary>
        public float x;
        /// <summary>
        /// Specifies the y-value of the vector component of the Quaternion.
        /// </summary>
        public float y;
        /// <summary>
        /// Specifies the z-value of the vector component of the Quaternion.
        /// </summary>
        public float z;
        /// <summary>
        /// Specifies the rotation component of the Quaternion.
        /// </summary>
        public float w;

        /// <summary>
        /// Returns a Quaternion representing no rotation. 
        /// </summary>
        public static Quaternion Identity
        {
            get { return new Quaternion(0, 0, 0, 1); }
        }

        /// <summary>A Quaternion representing the identity transform.</summary>
        public static readonly Quaternion identity = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);

        /// <summary>Constructs a Quaternion from four float values.</summary>
        /// <param name="x">The Quaternion x component.</param>
        /// <param name="y">The Quaternion y component.</param>
        /// <param name="z">The Quaternion z component.</param>
        /// <param name="w">The Quaternion w component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Quaternion(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }

        ///// <summary>Constructs a unit Quaternion from a float3x3 rotation matrix. The matrix must be orthonormal.</summary>
        ///// <param name="m">The float3x3 orthonormal rotation matrix.</param>
        //public Quaternion(Vector3 c0, Vector3 c1, Vector3 c2)
        //{
        //    var u = c0;
        //    var v = c1;
        //    var w = c2;

        //    uint u_sign = (asuint(u.x) & 0x80000000);
        //    float t = v.y + asfloat(asuint(w.z) ^ u_sign);
        //    uint4 u_mask = uint4((int)u_sign >> 31);
        //    uint4 t_mask = uint4(asint(t) >> 31);

        //    float tr = 1.0f + abs(u.x);

        //    uint4 sign_flips = uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

        //    value = float4(tr, u.y, w.x, v.z) + asfloat(asuint(float4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

        //    value = asfloat((asuint(value) & ~u_mask) | (asuint(value.zwxy) & u_mask));
        //    value = asfloat((asuint(value.wzyx) & ~t_mask) | (asuint(value) & t_mask));
        //    value = normalize(value);
        //}

        /// <summary>Constructs a unit Quaternion from an orthonormal float4x4 matrix.</summary>
        /// <param name="m">The float4x4 orthonormal rotation matrix.</param>
        //public Quaternion(float4x4 m)
        //{
        //    float4 u = m.c0;
        //    float4 v = m.c1;
        //    float4 w = m.c2;

        //    uint u_sign = (asuint(u.x) & 0x80000000);
        //    float t = v.y + asfloat(asuint(w.z) ^ u_sign);
        //    uint4 u_mask = uint4((int)u_sign >> 31);
        //    uint4 t_mask = uint4(asint(t) >> 31);

        //    float tr = 1.0f + abs(u.x);

        //    uint4 sign_flips = uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

        //    value = float4(tr, u.y, w.x, v.z) + asfloat(asuint(float4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

        //    value = asfloat((asuint(value) & ~u_mask) | (asuint(value.zwxy) & u_mask));
        //    value = asfloat((asuint(value.wzyx) & ~t_mask) | (asuint(value) & t_mask));

        //    value = normalize(value);
        //}

        /// <summary>
        /// Returns a Quaternion representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The Quaternion representing a rotation around an axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion AxisAngle(Vector3 axis, float angle)
        {
            Quaternion ans = Quaternion.identity;

            float halfAngle = angle * 0.5f;
            float s = (float)Math.Sin(halfAngle);
            float c = (float)Math.Cos(halfAngle);

            ans.x = axis.x * s;
            ans.y = axis.y * s;
            ans.z = axis.z * s;
            ans.w = c;
            return ans;
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerXYZ(Vector3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z - s.y * s.z * c.x,
                 s.y * c.x * c.z + s.x * s.z * c.y,
                 s.z * c.x * c.y - s.x * s.y * c.z,
                 c.x * c.y * c.z + s.y * s.z * s.x
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(-1.0f, 1.0f, -1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerXZY(Vector3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z + s.y * s.z * c.x,
                 s.y * c.x * c.z + s.x * s.z * c.y,
                 s.z * c.x * c.y - s.x * s.y * c.z,
                 c.x * c.y * c.z - s.y * s.z * s.x
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(1.0f, 1.0f, -1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerYXZ(Vector3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z - s.y * s.z * c.x,
                 s.y * c.x * c.z + s.x * s.z * c.y,
                 s.z * c.x * c.y + s.x * s.y * c.z,
                 c.x * c.y * c.z - s.y * s.z * s.x
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(-1.0f, 1.0f, 1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerYZX(Vector3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z - s.y * s.z * c.x,
                 s.y * c.x * c.z - s.x * s.z * c.y,
                 s.z * c.x * c.y + s.x * s.y * c.z,
                 c.x * c.y * c.z + s.y * s.z * s.x
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(-1.0f, -1.0f, 1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerZXY(Vector3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z + s.y * s.z * c.x,
                 s.y * c.x * c.z - s.x * s.z * c.y,
                 s.z * c.x * c.y - s.x * s.y * c.z,
                 c.x * c.y * c.z + s.y * s.z * s.x
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(1.0f, -1.0f, -1.0f, 1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerZYX(Vector3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            var halfAngle = xyz * 0.5f;
            var s = new Vector3((float)Math.Sin(halfAngle.x), (float)Math.Sin(halfAngle.y), (float)Math.Sin(halfAngle.z));
            var c = new Vector3((float)Math.Cos(halfAngle.x), (float)Math.Cos(halfAngle.y), (float)Math.Cos(halfAngle.z));
            return new Quaternion(
                 s.x * c.y * c.z + s.y * s.z * c.x,
                 s.y * c.x * c.z - s.x * s.z * c.y,
                 s.z * c.x * c.y + s.x * s.y * c.z,
                 c.x * c.y * c.z - s.y * s.x * s.z
                // float4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * float4(c.xyz, s.x) * float4(1.0f, -1.0f, 1.0f, -1.0f)
                );
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerXYZ(float x, float y, float z) { return EulerXYZ(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerXZY(float x, float y, float z) { return EulerXZY(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerYXZ(float x, float y, float z) { return EulerYXZ(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerYZX(float x, float y, float z) { return EulerYZX(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerZXY(float x, float y, float z) { return EulerZXY(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion EulerZYX(float x, float y, float z) { return EulerZYX(new Vector3(x, y, z)); }

        /// <summary>
        /// Returns a Quaternion constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A float3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in the specified order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Euler(Vector3 xyz, RotationOrder order = RotationOrder.ZXY)
        {
            switch (order)
            {
                case RotationOrder.XYZ:
                    return EulerXYZ(xyz);
                case RotationOrder.XZY:
                    return EulerXZY(xyz);
                case RotationOrder.YXZ:
                    return EulerYXZ(xyz);
                case RotationOrder.YZX:
                    return EulerYZX(xyz);
                case RotationOrder.ZXY:
                    return EulerZXY(xyz);
                case RotationOrder.ZYX:
                    return EulerZYX(xyz);
                default:
                    return Quaternion.identity;
            }
        }

        /// <summary>
        /// Returns a Quaternion constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The Quaternion representing the Euler angle rotation in the specified order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion Euler(float x, float y, float z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(new Vector3(x, y, z), order);
        }

        /// <summary>Returns a Quaternion that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The Quaternion representing a rotation around the x-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion RotateX(float angle)
        {
            float halfAngle = angle * 0.5f;
            float sina = (float)Math.Sin(halfAngle);
            float cosa = (float)Math.Cos(halfAngle);
            return new Quaternion(sina, 0.0f, 0.0f, cosa);
        }

        /// <summary>Returns a Quaternion that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The Quaternion representing a rotation around the y-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion RotateY(float angle)
        {
            float halfAngle = angle * 0.5f;
            float sina = (float)Math.Sin(halfAngle);
            float cosa = (float)Math.Cos(halfAngle);
            return new Quaternion(0.0f, sina, 0.0f, cosa);
        }

        /// <summary>Returns a Quaternion that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The Quaternion representing a rotation around the z-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Quaternion RotateZ(float angle)
        {
            float halfAngle = angle * 0.5f;
            float sina = (float)Math.Sin(halfAngle);
            float cosa = (float)Math.Cos(halfAngle);
            return new Quaternion(0.0f, 0.0f, sina, cosa);
        }

        /// <summary>
        /// Returns a Quaternion view rotation given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use float3x3.LookRotationSafe instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The Quaternion view rotation.</returns>
        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        //public static Quaternion LookRotation(Vector3 forward, Vector3 up)
        //{
        //    var t = Vector3.Normalize(Vector3.Cross(up, forward));
        //    return new Quaternion(float3x3(t, cross(forward, t), forward));
        //}

        /// <summary>
        /// Returns a Quaternion view rotation given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The Quaternion view rotation or the identity Quaternion.</returns>
        //public static Quaternion LookRotationSafe(float3 forward, float3 up)
        //{
        //    float forwardLengthSq = dot(forward, forward);
        //    float upLengthSq = dot(up, up);

        //    forward *= rsqrt(forwardLengthSq);
        //    up *= rsqrt(upLengthSq);

        //    float3 t = cross(up, forward);
        //    float tLengthSq = dot(t, t);
        //    t *= rsqrt(tLengthSq);

        //    float mn = min(min(forwardLengthSq, upLengthSq), tLengthSq);
        //    float mx = max(max(forwardLengthSq, upLengthSq), tLengthSq);

        //    bool accept = mn > 1e-35f && mx < 1e35f && isfinite(forwardLengthSq) && isfinite(upLengthSq) && isfinite(tLengthSq);
        //    return Quaternion(select(float4(0.0f, 0.0f, 0.0f, 1.0f), Quaternion(float3x3(t, cross(forward, t),forward)).value, accept));
        //}

        /// <summary>
        /// Flips the sign of each component of the quaternion.
        /// </summary>
        /// <param name="value">The source Quaternion.</param>
        /// <returns>The negated Quaternion.</returns>
        public static Quaternion Negate(Quaternion value)
        {
            Quaternion ans;

            ans.x = -value.x;
            ans.y = -value.y;
            ans.z = -value.z;
            ans.w = -value.w;

            return ans;
        }

        // <summary>
        /// Flips the sign of each component of the quaternion.
        /// </summary>
        /// <param name="value">The source Quaternion.</param>
        /// <returns>The negated Quaternion.</returns>
        public static Quaternion operator -(Quaternion value)
        {
            Quaternion ans;

            ans.x = -value.x;
            ans.y = -value.y;
            ans.z = -value.z;
            ans.w = -value.w;

            return ans;
        }

        /// <summary>
        /// Adds two Quaternions element-by-element.
        /// </summary>
        /// <param name="value1">The first source Quaternion.</param>
        /// <param name="value2">The second source Quaternion.</param>
        /// <returns>The result of adding the Quaternions.</returns>
        public static Quaternion operator +(Quaternion value1, Quaternion value2)
        {
            Quaternion ans;

            ans.x = value1.x + value2.x;
            ans.y = value1.y + value2.y;
            ans.z = value1.z + value2.z;
            ans.w = value1.w + value2.w;

            return ans;
        }

        /// <summary>
        /// Subtracts one Quaternion from another.
        /// </summary>
        /// <param name="value1">The first source Quaternion.</param>
        /// <param name="value2">The second Quaternion, to be subtracted from the first.</param>
        /// <returns>The result of the subtraction.</returns>
        public static Quaternion operator -(Quaternion value1, Quaternion value2)
        {
            Quaternion ans;

            ans.x = value1.x - value2.x;
            ans.y = value1.y - value2.y;
            ans.z = value1.z - value2.z;
            ans.w = value1.w - value2.w;

            return ans;
        }

        /// <summary>
        /// Multiplies two Quaternions together.
        /// </summary>
        /// <param name="value1">The Quaternion on the left side of the multiplication.</param>
        /// <param name="value2">The Quaternion on the right side of the multiplication.</param>
        /// <returns>The result of the multiplication.</returns>
        public static Quaternion operator *(Quaternion value1, Quaternion value2)
        {
            Quaternion ans;

            float q1x = value1.x;
            float q1y = value1.y;
            float q1z = value1.z;
            float q1w = value1.w;

            float q2x = value2.x;
            float q2y = value2.y;
            float q2z = value2.z;
            float q2w = value2.w;

            // cross(av, bv)
            float cx = q1y * q2z - q1z * q2y;
            float cy = q1z * q2x - q1x * q2z;
            float cz = q1x * q2y - q1y * q2x;

            float dot = q1x * q2x + q1y * q2y + q1z * q2z;

            ans.x = q1x * q2w + q2x * q1w + cx;
            ans.y = q1y * q2w + q2y * q1w + cy;
            ans.z = q1z * q2w + q2z * q1w + cz;
            ans.w = q1w * q2w - dot;

            return ans;
        }

        /// <summary>Returns the result of transforming a vector by a quaternion.</summary>
        /// <param name="q">The quaternion transformation.</param>
        /// <param name="v">The vector to transform.</param>
        /// <returns>The transformation of vector v by quaternion q.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3 operator *(Quaternion q, Vector3 v)
        {
            var aixs = new Vector3(q.x, q.y, q.z);
            var t = 2 * Vector3.Cross(aixs, v);
            return v + q.w * t + Vector3.Cross(aixs, t);
        }

        /// <summary>
        /// Multiplies a Quaternion by a scalar value.
        /// </summary>
        /// <param name="value1">The source Quaternion.</param>
        /// <param name="value2">The scalar value.</param>
        /// <returns>The result of the multiplication.</returns>
        public static Quaternion operator *(Quaternion value1, float value2)
        {
            Quaternion ans;

            ans.x = value1.x * value2;
            ans.y = value1.y * value2;
            ans.z = value1.z * value2;
            ans.w = value1.w * value2;

            return ans;
        }

        /// <summary>
        /// Creates the conjugate of a specified Quaternion.
        /// </summary>
        /// <param name="value">The Quaternion of which to return the conjugate.</param>
        /// <returns>A new Quaternion that is the conjugate of the specified one.</returns>
        public static Quaternion Conjugate(Quaternion value)
        {
            Quaternion ans;

            ans.x = -value.x;
            ans.y = -value.y;
            ans.z = -value.z;
            ans.w = value.w;

            return ans;
        }

        /// <summary>
        /// Returns the inverse of a Quaternion.
        /// </summary>
        /// <param name="value">The source Quaternion.</param>
        /// <returns>The inverted Quaternion.</returns>
        public static Quaternion Inverse(Quaternion value)
        {
            //  -1   (       a              -v       )
            // q   = ( -------------   ------------- )
            //       (  a^2 + |v|^2  ,  a^2 + |v|^2  )

            Quaternion ans;

            float ls = value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
            float invNorm = 1.0f / ls;

            ans.x = -value.x * invNorm;
            ans.y = -value.y * invNorm;
            ans.z = -value.z * invNorm;
            ans.w = value.w * invNorm;

            return ans;
        }

        /// <summary>
        /// Returns a boolean indicating whether the two given Quaternions are equal.
        /// </summary>
        /// <param name="value1">The first Quaternion to compare.</param>
        /// <param name="value2">The second Quaternion to compare.</param>
        /// <returns>True if the Quaternions are equal; False otherwise.</returns>
        public static bool operator ==(Quaternion value1, Quaternion value2)
        {
            return (value1.x == value2.x &&
                    value1.y == value2.y &&
                    value1.z == value2.z &&
                    value1.w == value2.w);
        }

        /// <summary>
        /// Returns a boolean indicating whether the two given Quaternions are not equal.
        /// </summary>
        /// <param name="value1">The first Quaternion to compare.</param>
        /// <param name="value2">The second Quaternion to compare.</param>
        /// <returns>True if the Quaternions are not equal; False if they are equal.</returns>
        public static bool operator !=(Quaternion value1, Quaternion value2)
        {
            return (value1.x != value2.x ||
                    value1.y != value2.y ||
                    value1.z != value2.z ||
                    value1.w != value2.w);
        }

        /// <summary>Returns true if the Quaternion is equal to a given Quaternion, false otherwise.</summary>
        /// <param name="x">The Quaternion to compare with.</param>
        /// <returns>True if the Quaternion is equal to the input, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(Quaternion q) { return this.x == q.x && this.y == q.y && this.z == q.z && this.w == q.w; }

        /// <summary>Returns whether true if the Quaternion is equal to a given Quaternion, false otherwise.</summary>
        /// <param name="x">The object to compare with.</param>
        /// <returns>True if the Quaternion is equal to the input, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object x) { return x is Quaternion converted && Equals(converted); }

        /// <summary>Returns a hash code for the Quaternion.</summary>
        /// <returns>The hash code of the Quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() { return x.GetHashCode() + y.GetHashCode() + z.GetHashCode() + w.GetHashCode(); }

        /// <summary>Returns a string representation of the Quaternion.</summary>
        /// <returns>The string representation of the Quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("Quaternion({0}f, {1}f, {2}f, {3}f)", x, y, z, w);
        }

        /// <summary>Returns a string representation of the Quaternion using a specified format and culture-specific format information.</summary>
        /// <param name="format">The format string.</param>
        /// <param name="formatProvider">The format provider to use during string formatting.</param>
        /// <returns>The formatted string representation of the Quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("Quaternion({0}f, {1}f, {2}f, {3}f)", x.ToString(format, formatProvider), y.ToString(format, formatProvider), z.ToString(format, formatProvider), w.ToString(format, formatProvider));
        }
    }
}
