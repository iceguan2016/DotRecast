
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromFaceToInnerVertices
    {
        public Face _fromFace;
        public Edge _nextEdge;
        public Vertex _resultVertex;

        public Face set_fromFace(Face value)
        {
            _fromFace = value;
            _nextEdge = _fromFace._edge;
            return value;
        }

        public Vertex next()
        {
            if (_nextEdge != null)
            {
                _resultVertex = _nextEdge.get_originVertex();
                _nextEdge = _nextEdge.get_nextLeftEdge();
                if (_nextEdge == _fromFace.get_edge())
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
