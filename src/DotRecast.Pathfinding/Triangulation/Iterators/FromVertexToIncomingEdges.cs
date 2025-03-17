using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromVertexToIncomingEdges
    {
        public Vertex _fromVertex;
        public Edge _nextEdge;
        public Edge _resultEdge;

        public Vertex set_fromVertex(Vertex value)
        {
            _fromVertex = value;
            _nextEdge = _fromVertex.get_edge();
            while (!_nextEdge.get_isReal())
            {
                _nextEdge = _nextEdge.get_rotLeftEdge();
            }
            return value;
        }


        public Edge next()
        {
            if (_nextEdge != null)
            {
                // 注：_nextEdge.get_rotLeftEdge得到的是下一个(逆时针方向)从_fromVertex发出的edge
                // 这里需要的是IncommingEdge，所以要取oppositeEdge
                _resultEdge = _nextEdge.get_oppositeEdge();
                do
                {
                    _nextEdge = _nextEdge.get_rotLeftEdge();
                    if (_nextEdge == _fromVertex.get_edge())
                    {
                        _nextEdge = null;
                        break;
                    }
                } while ((!_nextEdge.get_isReal()));
            }
            else
            {
                _resultEdge = null;
            }
            return _resultEdge;
        }
    }
}
