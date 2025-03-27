
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Main
{
    public class PathfindingMoudle
    {
        public static void StartupModule()
        {
            // Triangulation
            Vertex.INC = 0;
            Face.INC = 0;
            Edge.INC = 0;
            ConstraintSegment.INC = 0;
            ConstraintShape.INC = 0;
            Object.INC = 0;

            // 
        }

	    public static void ShutdownModule()
        {
            
        }
    }
}
