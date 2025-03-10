
namespace DotRecast.Pathfinding.Util
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
            return new FixMath.F64Vec2(v.X, v.Y);
        }
    }
}
