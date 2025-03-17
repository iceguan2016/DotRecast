using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromVertexToNeighbourVertices
    {
        public Vertex _fromVertex;
        public Edge _nextEdge;
        public Vertex _resultVertex;

        public Vertex set_fromVertex(Vertex value)
        {
            _fromVertex = value;
            _nextEdge = _fromVertex.get_edge();
            return value;
        }

        public Vertex next()
        {
            if (_nextEdge != null)
            {
                _resultVertex = _nextEdge.get_oppositeEdge().get_originVertex();
                do
                {
                    _nextEdge = _nextEdge.get_rotLeftEdge();
                } while ((!_nextEdge.get_isReal()));

                if (_nextEdge == _fromVertex.get_edge())
                    _nextEdge = null;
            }
            else
            {
                _resultVertex = null;
            }

            return _resultVertex;
        }
    }
}
