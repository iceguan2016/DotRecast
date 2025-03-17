using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromMeshToVertices
    {
        public Mesh _fromMesh;
        public int _currIndex;
        public Vertex _resultVertex;

        public Mesh set_fromMesh(Mesh value)
        {
            _fromMesh = value;
            _currIndex = 0;
            return value;
        }

        public Vertex next()
        {
            do
            {
                if (_currIndex < _fromMesh._vertices.Count)
                {
                    _resultVertex = _fromMesh._vertices[_currIndex];
                    _currIndex++;
                }
                else
                {
                    _resultVertex = null;
                    break;
                }
            } while ((!_resultVertex.get_isReal()));
            return _resultVertex;
        }
    }
}
