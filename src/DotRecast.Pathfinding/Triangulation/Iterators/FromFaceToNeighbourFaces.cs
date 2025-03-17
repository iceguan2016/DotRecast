
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromFaceToNeighbourFaces
    {
        public Face _fromFace;
        public Edge _nextEdge;
        public Face _resultFace;

        public Face set_fromFace(Face value)
        {
            _fromFace = value;
            _nextEdge = _fromFace.get_edge();
            return value;
        }

        public Face next()
        {
            if (_nextEdge != null)
            {
                do
                {
                    _resultFace = _nextEdge.get_rightFace();
                    _nextEdge = _nextEdge.get_nextLeftEdge();
                    if (_nextEdge == _fromFace.get_edge())
                    {
                        _nextEdge = null;
                        if (!_resultFace.get_isReal())
                            _resultFace = null;
                        break;
                    }
                } while ((!_resultFace.get_isReal()));
            }
            else
            {
                _resultFace = null;
            }
            return _resultFace;
        }
    }
}
