
using System.Collections.Generic;
using System.Linq;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Math;

namespace Pathfinding.Triangulation.AI
{
    public class PathFinder
    {
        public PathFinder()
        {
            this.astar = new AStar();
            this.funnel = new Funnel();
            this.listFaces = new List<Face>();
            this.listEdges = new List<Edge>();
        }

        // public global::hxDaedalus.ai.EntityAI entity;

        public Mesh _mesh;

        public AStar astar;

        public Funnel funnel;

        public FixMath.F64 radius;

        public List<Face> listFaces;

        public List<Edge> listEdges;

        public bool isPartial;

        public virtual void dispose()
        {
            this._mesh = null;
            this.astar.dispose();
            this.astar = null;
            this.funnel.dispose();
            this.funnel = null;
            this.listEdges = null;
            this.listFaces = null;
            this.isPartial = false;
        }


        public Mesh get_mesh()
        {
            return this._mesh;
        }


        public Mesh set_mesh(Mesh mesh)
        {
            this._mesh = mesh;
            this.astar.set_mesh(this._mesh);
            return mesh;
        }

        public void findPath(FixMath.F64 fromX, FixMath.F64 fromY, FixMath.F64 toX, FixMath.F64 toY, FixMath.F64 radius, int iterMaxTimes, List<FixMath.F64> resultPath)
        {
            resultPath.Clear();
            //Debug.assertFalse(_mesh == null, "Mesh missing");
            //Debug.assertFalse(entity == null, "Entity missing");

            if (Geom2D.isCircleIntersectingAnyConstraint(toX, toY, radius, _mesh))
                return;

            astar.set_radius(radius);
            funnel.set_radius(radius);

            listFaces.Clear();
            listEdges.Clear();
            isPartial = false;
            astar.findPath(fromX, fromY, toX, toY, iterMaxTimes, listFaces, listEdges, out isPartial);
            if (listFaces.Count == 0)
            {
                // Debug.trace("PathFinder listFaces.length == 0");
                return;
            }

            if (isPartial)
            {
                // 修正(toX, toY)
                var closest = Geom2D.closestPointToFace(toX, toY, listFaces.Last());
                funnel.findPath(fromX, fromY, closest.X, closest.Y, listFaces, listEdges, resultPath);
            }
            else
            {
                funnel.findPath(fromX, fromY, toX, toY, listFaces, listEdges, resultPath);
            }
        }
    }
}
