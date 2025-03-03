using System;

namespace FixMath
{
    // Unity使用左手坐标系
    public struct F64Matrix
    {
        // The Get function accesses the matrix in std math convention
        // m0,0 m0,1 m0,2
        // m1,0 m1,1 m1,2
        // m2,0 m2,1 m2,2

        // The floats are laid out:
        // m0   m3   m6
        // m1   m4   m7
        // m2   m5   m8
        public FixMath.F64 M00, M01, M02, M03;
        public FixMath.F64 M10, M11, M12, M13;
        public FixMath.F64 M20, M21, M22, M23;
        public FixMath.F64 M30, M31, M32, M33;

        public enum eAxis
        {
            None,
            X,
            Y,
            Z,
        };

        public FixMath.F64Vec3 GetScaledAxis(eAxis InAxis )
        {
	        switch (InAxis )
	        {
	        case eAxis.X:
		        return new FixMath.F64Vec3(M00, M10, M20);
	        case eAxis.Y:
		        return new FixMath.F64Vec3(M01, M11, M21);
	        case eAxis.Z:
		        return new FixMath.F64Vec3(M02, M12, M22);
	        default:
		        return FixMath.F64Vec3.Zero;
	        }
        }

        public void SetScaledAxis(eAxis InAxis, FixMath.F64Vec3 InDir)
        {
            switch (InAxis )
	        {
	        case eAxis.X:
		        { M00 = InDir.X; M10 = InDir.Y; M20 = InDir.Z; } break;
	        case eAxis.Y:
		        { M01 = InDir.X; M11 = InDir.Y; M21 = InDir.Z; } break;
	        case eAxis.Z:
		        { M02 = InDir.X; M12 = InDir.Y; M22 = InDir.Z; } break;
            default: break;
            }
        }

        public void SetIdentity()
        {
            this[0, 0] = F64.One; this[0, 1] = F64.Zero; this[0, 2] = F64.Zero; this[0, 3] = F64.Zero;
            this[1, 0] = F64.Zero; this[1, 1] = F64.One; this[1, 2] = F64.Zero; this[1, 3] = F64.Zero;
            this[2, 0] = F64.Zero; this[2, 1] = F64.Zero; this[2, 2] = F64.One; this[2, 3] = F64.Zero;
            this[3, 0] = F64.Zero; this[3, 1] = F64.Zero; this[3, 2] = F64.Zero; this[3, 3] = F64.One;
        }

        public void SetBasis(F64Vec3 inX, F64Vec3 inY, F64Vec3 inZ)
        {
            this[0, 0] = inX[0];    this[0, 1] = inY[0];    this[0, 2] = inZ[0];
            this[1, 0] = inX[1];    this[1, 1] = inY[1];    this[1, 2] = inZ[1];
            this[2, 0] = inX[2];    this[2, 1] = inY[2];    this[2, 2] = inZ[2];
        }

        public void SetTranslation(F64Vec3 inTranlation)
        {
            this[0, 3] = inTranlation.X;
            this[1, 3] = inTranlation.Y;
            this[2, 3] = inTranlation.Z;
        }

        public F64Vec3 GetTranslation()
        {
            return new F64Vec3(this[0, 3], this[1, 3], this[2, 3]);
        }

        public F64 this[int i, int j] { 
            get
            {
                var index = i + j * 4;
                return index switch
                {
                    0 => M00,
                    1 => M10,
                    2 => M20,
                    3 => M30,
                    4 => M01,
                    5 => M11,
                    6 => M21,
                    7 => M31,
                    8 => M02,
                    9 => M12,
                    10 => M22,
                    11 => M32,
                    12 => M03,
                    13 => M13,
                    14 => M23,
                    15 => M33,
                    _ => throw new IndexOutOfRangeException("Invalid matrix index!"),
                };
            }

            set
            {
                var index = i + j * 4;
                switch (index)
                {
                    case 0:
                        M00 = value;
                        break;
                    case 1:
                        M10 = value;
                        break;
                    case 2:
                        M20 = value;
                        break;
                    case 3:
                        M30 = value;
                        break;
                    case 4:
                        M01 = value;
                        break;
                    case 5:
                        M11 = value;
                        break;
                    case 6:
                        M21 = value;
                        break;
                    case 7:
                        M31 = value;
                        break;
                    case 8:
                        M02 = value;
                        break;
                    case 9:
                        M12 = value;
                        break;
                    case 10:
                        M22 = value;
                        break;
                    case 11:
                        M32 = value;
                        break;
                    case 12:
                        M03 = value;
                        break;
                    case 13:
                        M13 = value;
                        break;
                    case 14:
                        M23 = value;
                        break;
                    case 15:
                        M33 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("Invalid matrix index!");
                }
            }
        }

        // 
        public static bool FromLookRotation(F64Vec3 viewVec, F64Vec3 upVec, out F64Matrix m)
        {
            m = new F64Matrix();
            F64Vec3 z = viewVec;
            // compute u0
            F64 mag = F64Vec3.LengthFast(z);
            if (mag < F64.Epsilon)
            {
                m.SetIdentity();
                return false;
            }
            z /= mag;

            F64Vec3 x = F64Vec3.Cross(upVec, z);
            mag = F64Vec3.LengthFast(x);
            if (mag < F64.Epsilon)
            {
                m.SetIdentity();
                return false;
            }
            x /= mag;

            F64Vec3 y = F64Vec3.Cross(z, x);
            if (F64.Abs(F64Vec3.Dot(y, y) - F64.One) > F64.Epsilon)
            {
                m.SetIdentity();
                return false;
            }

            m.SetBasis(x, y, z);
            return true;
        }

        // 转换为四元数
        public F64Quat ToQuat()
        {
            F64Quat q = new F64Quat();
            F64 fTrace = this[0, 0] + this[1, 1] + this[2, 2];
            F64 fRoot;

            if (fTrace > F64.Zero)
            {
                // |w| > 1/2, may as well choose w > 1/2
                fRoot = F64.Sqrt(fTrace + F64.One);   // 2w
                q.W = F64.Half * fRoot;
                fRoot = F64.Half / fRoot;  // 1/(4w)
                q.X = (this[2, 1] - this[1, 2]) * fRoot;
                q.Y = (this[0, 2] - this[2, 0]) * fRoot;
                q.Z = (this[1, 0] - this[0, 1]) * fRoot;
            }
            else
            {
                // |w| <= 1/2
                int[] s_iNext = new int[3] { 1, 2, 0 };
                int i = 0;
                if (this[1, 1] > this[0, 0])
                    i = 1;
                if (this[2, 2] > this[i, i])
                    i = 2;
                int j = s_iNext[i];
                int k = s_iNext[j];

                fRoot = F64.Sqrt(this[i, i] - this[j, j] - this[k, k] + F64.One);
                F64[] apkQuat = new F64[3];
                // Common.DebugUtils.Assert(fRoot > F64.Zero);
                apkQuat[i] = F64.Half * fRoot;
                fRoot = F64.Half / fRoot;
                q.W = (this[k, j] - this[j, k]) * fRoot;
                apkQuat[j] = (this[j, i] + this[i, j]) * fRoot;
                apkQuat[k] = (this[k, i] + this[i, k]) * fRoot;

                q.X = apkQuat[0];
                q.Y = apkQuat[1]; 
                q.Z = apkQuat[2];
            }
            q = F64Quat.Normalize(q);
            return q;
        }
    }
}
