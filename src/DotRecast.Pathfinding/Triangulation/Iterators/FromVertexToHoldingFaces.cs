
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Iterators
{
    //  implements an iterator that traverses a sequence of faces connected by edges in a cyclical manner (likely around a vertex).
    //  It has logic to handle invalid or irrelevant "non-real" faces and terminates when a complete cycle is detected or the end of the sequence is reached.
    //  遍历和顶点v相连的所有face(f0, f1, f2, f3)
    //                       ________
    //                      /`.  f0 .'\
    //                     /f1 `. .'f3 \
    //                     \   .'v`.   /
    //                      \.'__f2_`./
    public class FromVertexToHoldingFaces
    {
        public FromVertexToHoldingFaces()
        {
            
        }

        public Vertex _fromVertex;

        public Edge _nextEdge;

        public Face _resultFace;

        public Vertex set_fromVertex(Vertex @value)
        {
            this._fromVertex = @value;
            this._nextEdge = this._fromVertex.get_edge();
            return @value;
        }


        public Face next()
        {
            if ((this._nextEdge != null))
            {
                do
                {
                    this._resultFace = this._nextEdge.get_leftFace();
                    this._nextEdge = this._nextEdge.get_rotLeftEdge();
                    if ((this._nextEdge == this._fromVertex.get_edge()))
                    {
                        this._nextEdge = null;
                        if (!(this._resultFace.get_isReal()))
                        {
                            this._resultFace = null;
                        }

                        break;
                    }

                }
                while (!(this._resultFace.get_isReal()));
            }
            else
            {
                this._resultFace = null;
            }

            return this._resultFace;
        }
    }
}
