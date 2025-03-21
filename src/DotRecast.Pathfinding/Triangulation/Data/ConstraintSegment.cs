using System.Collections.Generic;
using Pathfinding.Util;

namespace Pathfinding.Triangulation.Data
{
    public class ConstraintSegment
    {
        static ConstraintSegment()
        {
            ConstraintSegment.INC = 0;
        }

        public ConstraintSegment()
        {
            _id = INC;
            INC++;
            _edges = new List<Edge>();
        }


        public static int INC;

        public ConstraintShape fromShape;
        public int _id;
        public List<Edge> _edges;

        public int get_id()
        {
            return this._id;
        }

        public void addEdge(Edge edge)
        {
            if (!_edges.Contains(edge) && !_edges.Contains(edge._oppositeEdge))
            {
                Debug.LogToFile($"ConstraintSegment addEdge id:{_id}, edge_id:{edge._id}");
                _edges.Add(edge);
            }
        }


        public void removeEdge(Edge edge)
        {
            var index = _edges.IndexOf(edge);
            if (index == -1)
                index = _edges.IndexOf(edge._oppositeEdge);
            if (index != -1)
            {
                Debug.LogToFile($"ConstraintSegment removeEdge id:{_id}, edge_id:{edge._id}, index:{index}");
                _edges.RemoveAt(index);
            }
        }


        public List<Edge> get_edges()
        {
            return this._edges;
        }


        public void dispose()
        {
            this._edges = null;
            this.fromShape = null;
        }


        public string toString()
        {
            return "seg_id " + this._id;
        }
    }
}
