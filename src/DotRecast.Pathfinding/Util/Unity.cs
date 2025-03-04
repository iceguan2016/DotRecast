
namespace DotRecast.Pathfinding.Util
{
    public static class Unity
    {
        public static UnityEngine.Vector3 Cast(this FixMath.F64Vec3 v)
        { 
            return new UnityEngine.Vector3(v.X.Float, v.Y.Float, v.Z.Float);
        }

        public static FixMath.F64Vec3 Cast(this UnityEngine.Vector3 v)
        {
            return FixMath.F64Vec3.FromFloat(v.x, v.y, v.z);
        }

        public static UnityEngine.Color Cast(this FixMath.F64Vec3 c, FixMath.F64 a)
        {
            return new UnityEngine.Color(c.X.Float, c.Y.Float, c.Z.Float, a.Float);
        }
    }
}
