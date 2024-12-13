using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;

namespace DotRecast.Pathfinding.Crowds
{
    public struct BoundarySegement
    {
        public Vector3 Start;
        public Vector3 End;
    }

    // The query interface for querying the boundary information within
    // the nearby range is implemented by the specific module itself
    public interface ILocalBoundaryQuerier
    {
        // Query the boundary information in the nearby range (those closer are returned first)
        int QueryBoundaryInCircle(float inRadius, BoundarySegement[] outResults, int inMaxBuffSize);
    }
}
