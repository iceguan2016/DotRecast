
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromMeshToFaces
    {
        public Mesh _fromMesh;
        public int _currIndex;
        public Face _resultFace;

        public Mesh set_fromMesh(Mesh value)
        {
            _fromMesh = value;
            _currIndex = 0;
            return value;
        }

        public Face next()
        {
            do
            {
                if (_currIndex < _fromMesh._faces.Count)
                {
                    _resultFace = _fromMesh._faces[_currIndex];
                    _currIndex++;
                }
                else
                {
                    _resultFace = null;
                    break;
                }
            } while ((!_resultFace.get_isReal()));
            return _resultFace;
        }
    }
}
