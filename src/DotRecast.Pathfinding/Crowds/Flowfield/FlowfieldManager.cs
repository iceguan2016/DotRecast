using Pathfinding.Triangulation.Data;
using SharpSteer2.Obstacles;

namespace Pathfinding.Crowds.Flowfield
{
    // 管理Flowfield
    public class FlowfieldManager
    {
        public FixMath.F64Vec3 BoundsMin { get; private set; }
        public FixMath.F64Vec3 BoundsMax { get; private set; }

        public void SetNavMesh(Mesh mesh)
        {
                
        }

        public FixMath.F64Vec3 QueryReferenceDirection(
            MovableEntity vehicle, ref PathIntersection intersection)
        { 
            return FixMath.F64Vec3.Zero;
        }

        FlowfieldTile FindOrCreateTile()
        {
            return null;
        }
    }
}
