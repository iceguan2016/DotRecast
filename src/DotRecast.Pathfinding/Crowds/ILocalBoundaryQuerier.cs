
using SharpSteer2;

namespace Pathfinding.Crowds
{
    public struct BoundarySegement
    {
        public FixMath.F64Vec3 Start;
        public FixMath.F64Vec3 End;
    }

    // The query interface for querying the boundary information within
    // the nearby range is implemented by the specific module itself
    public interface ILocalBoundaryQuerier
    {
        // Query the boundary information in the nearby range (those closer are returned first)
        int QueryBoundaryInCircle(IVehicle vehicle, FixMath.F64 inRadius, BoundarySegement[] outResults);
    }
}
