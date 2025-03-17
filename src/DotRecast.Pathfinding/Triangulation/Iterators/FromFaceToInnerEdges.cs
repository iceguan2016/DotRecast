
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    public class FromFaceToInnerEdges
    {
        public Face _fromFace;

        public Edge _nextEdge;

        public Edge _resultEdge;

        public Face set_fromFace(Face @value)
        {
            this._fromFace = @value;
            this._nextEdge = this._fromFace.get_edge();
            return @value;
        }


        public Edge next()
        {
            if ((this._nextEdge != null))
            {
                this._resultEdge = this._nextEdge;
                this._nextEdge = this._nextEdge.get_nextLeftEdge();
                if ((this._nextEdge == this._fromFace.get_edge()))
                {
                    this._nextEdge = null;
                }
            }
            else
            {
                this._resultEdge = null;
            }

            return this._resultEdge;
        }
    }
}
