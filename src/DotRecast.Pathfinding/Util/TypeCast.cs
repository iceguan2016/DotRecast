
namespace Pathfinding.Util
{
    public static class TypeCast
    {
        public static UnityEngine.Vector3 Cast(this FixMath.F64Vec3 v)
        { 
            return new UnityEngine.Vector3(v.X.Float, v.Y.Float, v.Z.Float);
        }

        public static FixMath.F64Vec3 Cast(this UnityEngine.Vector3 v)
        {
            return FixMath.F64Vec3.FromFloat(v.x, v.y, v.z);
        }

        public static UnityEngine.Vector2 Cast(this FixMath.F64Vec2 v)
        {
            return new UnityEngine.Vector2(v.X.Float, v.Y.Float);
        }

        public static FixMath.F64Vec2 Cast(this UnityEngine.Vector2 v)
        {
            return FixMath.F64Vec2.FromFloat(v.x, v.y);
        }

        public static UnityEngine.Color Cast(this FixMath.F64Vec3 c, FixMath.F64 a)
        {
            return new UnityEngine.Color(c.X.Float, c.Y.Float, c.Z.Float, a.Float);
        }

        public static FixMath.F64Vec3 Cast(this FixMath.F64Vec2 v, FixMath.F64 y)
        {
            return new FixMath.F64Vec3(v.X, y, v.Y);
        }

        public static FixMath.F64Vec2 Cast2D(this FixMath.F64Vec3 v)
        {
            return new FixMath.F64Vec2(v.X, v.Z);
        }

        public static FixMath.NET.Fix64 ToF64(this FixMath.F32 InF32)
        {
            return FixMath.NET.Fix64.FromRaw(((long)InF32.Raw) << 16);
        }
        public static FixMath.NET.Fix64 ToF64(this FixMath.F64 InF64)
        {
            return FixMath.NET.Fix64.FromRaw(InF64.Raw);
        }
        public static FixMath.F32 ToF32(this FixMath.NET.Fix64 InF64)
        {
            return FixMath.F32.FromRaw((int)(InF64.RawValue >> 16));
        }
        public static FixMath.F64 ToF64(this FixMath.NET.Fix64 InF64)
        {
            return FixMath.F64.FromRaw(InF64.RawValue);
        }

        public static Volatile.VoltVector2 ToVoltVec2(this FixMath.F64Vec3 InVec3)
        {
            return new Volatile.VoltVector2(InVec3.X.ToF64(), InVec3.Z.ToF64());
        }

        public static Volatile.VoltVector2 ToVoltVec2(this FixMath.F64Vec2 InVec2)
        {
            return new Volatile.VoltVector2(InVec2.X.ToF64(), InVec2.Y.ToF64());
        }

        public static FixMath.F64Vec3 ToVec3(this Volatile.VoltVector2 InVec2, FixMath.F64 InHeight)
        {
            return new FixMath.F64Vec3(InVec2.x.ToF64(), InHeight, InVec2.y.ToF64());
        }
    }
}
