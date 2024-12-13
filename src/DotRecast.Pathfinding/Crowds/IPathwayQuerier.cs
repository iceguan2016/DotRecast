using System;
using System.Collections.Generic;
using System.Text;
using SharpSteer2;
using SharpSteer2.Pathway;

namespace DotRecast.Pathfinding.Crowds
{
    // Path finder for vehicle
    public interface IPathwayQuerier
    {
        PolylinePathway FindPath(IVehicle vehicle);
    }
}
