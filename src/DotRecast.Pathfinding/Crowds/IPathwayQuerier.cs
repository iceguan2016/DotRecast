using SharpSteer2;
using SharpSteer2.Pathway;

namespace Pathfinding.Crowds
{
    // Path finder for vehicle
    public interface IPathwayQuerier
    {
        PolylinePathway FindPath(IVehicle vehicle, FixMath.F64Vec3 target);
    }
}
