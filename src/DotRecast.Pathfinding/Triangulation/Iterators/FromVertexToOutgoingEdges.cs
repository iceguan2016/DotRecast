using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromVertexToOutgoingEdges
    {
        public Vertex _fromVertex;
        public Edge _nextEdge;
        public Edge _resultEdge;

        public bool realEdgesOnly = true;


        public Vertex set_fromVertex(Vertex value)
        {
            _fromVertex = value;
            _nextEdge = _fromVertex.get_edge();
            while (realEdgesOnly && !_nextEdge.get_isReal())
            {
                _nextEdge = _nextEdge.get_rotLeftEdge();
            }
            return value;
        }

        public Edge next()
        {
            if (_nextEdge != null)
            {
                _resultEdge = _nextEdge;
                do
                {
                    _nextEdge = _nextEdge.get_rotLeftEdge();
                    if (_nextEdge == _fromVertex.get_edge())
                    {
                        _nextEdge = null;
                        break;
                    }
                } while ((realEdgesOnly && !_nextEdge.get_isReal()));
            }
            else
            {
                _resultEdge = null;
            }

            return _resultEdge;
        }
    }
}
