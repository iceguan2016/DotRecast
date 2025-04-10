
using System.Collections.Generic;

namespace Pathfinding.Triangulation.Data
{
    // https://jerryyin.info/geometry-processing-algorithms/half-edge/
    // face对应edge按逆时针方向走，leftEdge是对应face的innerEdge

    // _isReal : 标记edge是否是原始输入数据的edge（false表示是后面生成新增的edge）
    // _isConstrained : 标记顶点是否是约束边
    // _originVertex : edge的起始顶点
    // _oppositeEdge : edge对象边
    // _nextLeftEdge : face的innerEdge中与本edge相连的下一个edge(逆时针方向)
    // _leftFace : 本edge作为innerEdge所属的face

    /*
                                    .    
                                  .'  `.  
                                .'      `. 
       prevLeftEdge(inner)    .'          `.
       rotLeftEdge(out)     .'              `. nextLeftEdge(inner)
                          .'      leftFace    `.
                        .'                      `.
         originVertex .'----------edge----------->.'destinationVertex
                      `.<---------opp_edge------.'
                        `.                    .'
                          `.     rightFace  .'
         prevRightEdge(out) `.            .'nextRightEdge(out) = opp_edge->nextLeftEdge->nextLeftEdge->opp_edge
         rotRightEdge(inner)  `.        .'
                                `.    .' 
                                   `.'   
     rotLeftEdge: 得到下一个从originVertex发出去的edge (逆时针方向)
     rotRightEdge: 得到前一个从originVertex发出去的edge (逆时针方向)
     */

    public class Edge
    {
        static Edge()
        {
            Edge.INC = 0;
        }


        public Edge()
        {
            colorDebug = -1;

            this._id = INC;
            ++INC;

            fromConstraintSegments = new List<ConstraintSegment>();
        }


        public static int INC;

        public List<ConstraintSegment> fromConstraintSegments;

        public int _id;

        public bool _isReal;

        public bool _isConstrained;

        // 如果是约束边，标记该edge是否和元素shape中的边反向
        public bool _isReversed = false;

        public Vertex _originVertex;

        public Edge _oppositeEdge;

        public Edge _nextLeftEdge;

        public Face _leftFace;

        public int colorDebug;

        public int get_id()
        {
            return this._id;
        }


        public bool get_isReal()
        {
            return this._isReal;
        }


        public bool get_isConstrained()
        {
            return this._isConstrained;
        }


        public void setDatas(Vertex originVertex, Edge oppositeEdge, Edge nextLeftEdge, Face leftFace, bool isReal = true, bool isConstrained = false)
        {
            this._isConstrained = isConstrained;
            this._isReal = isReal;
            this._originVertex = originVertex;
            this._oppositeEdge = oppositeEdge;
            this._nextLeftEdge = nextLeftEdge;
            this._leftFace = leftFace;
        }


        public void addFromConstraintSegment(ConstraintSegment segment)
        {
            if (!fromConstraintSegments.Contains(segment))
            { 
                fromConstraintSegments.Add(segment);
            }
        }


        public void removeFromConstraintSegment(ConstraintSegment segment)
        {
            fromConstraintSegments.Remove(segment);
        }


        public Vertex set_originVertex(Vertex value)
        {
            this._originVertex = value;
            return value;
        }


        public Edge set_nextLeftEdge(Edge value)
        {
            this._nextLeftEdge = value;
            return value;
        }


        public Face set_leftFace(Face value)
        {
            this._leftFace = value;
            return value;
        }


        public bool set_isConstrained(bool value)
        {
            this._isConstrained = value;
            return value;
        }


        public void dispose()
        {
            this._originVertex = null;
            this._oppositeEdge = null;
            this._nextLeftEdge = null;
            this._leftFace = null;
            this.fromConstraintSegments.Clear();
        }


        public Vertex get_originVertex()
        {
            return this._originVertex;
        }


        public Vertex get_destinationVertex()
        {
            return this.get_oppositeEdge().get_originVertex();
        }


        public Edge get_oppositeEdge()
        {
            return this._oppositeEdge;
        }


        public Edge get_nextLeftEdge()
        {
            return this._nextLeftEdge;
        }


        public Edge get_prevLeftEdge()
        {
            return this._nextLeftEdge.get_nextLeftEdge();
        }


        public Edge get_nextRightEdge()
        {
            return this._oppositeEdge.get_nextLeftEdge().get_nextLeftEdge().get_oppositeEdge();
        }


        public Edge get_prevRightEdge()
        {
            return this._oppositeEdge.get_nextLeftEdge().get_oppositeEdge();
        }


        public Edge get_rotLeftEdge()
        {
            return this._nextLeftEdge.get_nextLeftEdge().get_oppositeEdge();
        }


        public Edge get_rotRightEdge()
        {
            return this._oppositeEdge.get_nextLeftEdge();
        }


        public Face get_leftFace()
        {
            return this._leftFace;
        }


        public Face get_rightFace()
        {
            return this._oppositeEdge.get_leftFace();
        }


        public string toString()
        {
            return "edge " + get_originVertex().get_id() + " - " + get_destinationVertex().get_id();
        }
    }
}
