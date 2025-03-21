
using System;
using System.Collections.Generic;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Triangulation.Math;

namespace Pathfinding.Triangulation.AI
{
    public enum EFindPathResult
    {
        Success = 0,
        Failed,
        Progress
    }
    public class AStar
    {
        public AStar()
        {
            iterEdge = new FromFaceToInnerEdges();
        }

        private FixMath.F64 _radius;

        private Mesh _mesh;

        private Dictionary<Face, bool> closedFaces;
        private List<Face> sortedOpenedFaces;
        private Dictionary<Face, bool> openedFaces;
        private Dictionary<Face, Edge> entryEdges;
        private Dictionary<Face, FixMath.F64> entryX;
        private Dictionary<Face, FixMath.F64> entryY;
        private Dictionary<Face, FixMath.F64> scoreF;
        private Dictionary<Face, FixMath.F64> scoreG;
        private Dictionary<Face, FixMath.F64> scoreH;
        private Dictionary<Face, Face> predecessor;
        private FromFaceToInnerEdges iterEdge;
        private FixMath.F64 radiusSquared;
        private FixMath.F64 diameter;
        private FixMath.F64 diameterSquared;
        private Face fromFace;
        private Face toFace;
        private Face curFace;

        public void dispose()
        {
            this._mesh = null;
            this.closedFaces = null;
            this.sortedOpenedFaces = null;
            this.openedFaces = null;
            this.entryEdges = null;
            this.entryX = null;
            this.entryY = null;
            this.scoreF = null;
            this.scoreG = null;
            this.scoreH = null;
            this.predecessor = null;
        }


        public FixMath.F64 get_radius()
        {
            return this._radius;
        }


        public FixMath.F64 set_radius(FixMath.F64 rdius)
        {
            this._radius = rdius;
            this.radiusSquared = (this._radius * this._radius);
            this.diameter = (this._radius * 2);
            this.diameterSquared = (this.diameter * this.diameter);
            return rdius;
        }

        public Mesh set_mesh(Mesh mesh)
        {
            this._mesh = mesh;
            return mesh;
        }

        public EFindPathResult initialPath(FixMath.F64 fromX, FixMath.F64 fromY
                                    , FixMath.F64 toX, FixMath.F64 toY)
        {
            return EFindPathResult.Failed;
        }

        public EFindPathResult updatePath(int stepNum
                            , List<Face> resultListFaces
                            , List<Edge> resultListEdges)
        {
            return EFindPathResult.Failed;
        }

        public void findPath(FixMath.F64 fromX, FixMath.F64 fromY
                            , FixMath.F64 toX, FixMath.F64 toY
                            , List<Face> resultListFaces
                            , List<Edge> resultListEdges) 
        {
            //Debug.trace("findPath");
            closedFaces = new Dictionary<Face, bool>();
            sortedOpenedFaces = new List<Face>();
            openedFaces = new Dictionary<Face, bool>();
            entryEdges = new Dictionary<Face, Edge>();
            entryX = new Dictionary<Face, FixMath.F64>();
            entryY = new Dictionary<Face, FixMath.F64>();
            scoreF = new Dictionary<Face, FixMath.F64>();
            scoreG = new Dictionary<Face, FixMath.F64>();
            scoreH = new Dictionary<Face, FixMath.F64>();
            predecessor = new Dictionary<Face, Face>();
        
            Intersection loc;
            Edge locEdge;
            Vertex locVertex;
            FixMath.F64 distance;
            FixMath.F64Vec2 p1;
            FixMath.F64Vec2 p2;
            FixMath.F64Vec2 p3;
            //
            loc = Geom2D.locatePosition(fromX, fromY, _mesh);
            {
                if (loc is Intersection_EVertex v0)
                {
                    locVertex = v0.vertex;
                    return;
                }
                else if (loc is Intersection_EEdge e0)
                {
                    locEdge = e0.edge;
                    return;
                }
                else if (loc is Intersection_EFace f0)
                {
                    fromFace = f0.face;
                }
            }
            

            loc = Geom2D.locatePosition(toX, toY, _mesh);
            {
                if (loc is Intersection_EVertex v1)
                {
                    locVertex = v1.vertex;
                    toFace = locVertex._edge.get_leftFace();
                }
                else if (loc is Intersection_EEdge e1)
                {
                    locEdge = e1.edge;
                    toFace = locEdge.get_leftFace();
                }
                else if (loc is Intersection_EFace f1)
                {
                    toFace = f1.face;
                }
            }

            /*
            fromFace.colorDebug = 0xFF0000;
            toFace.colorDebug = 0xFF0000;
            Debug.trace( "from face: " + fromFace );
            Debug.trace( "to face: " + toFace );
            */

            sortedOpenedFaces.Add(fromFace);
            entryEdges[fromFace] = null;
            entryX[fromFace] = fromX;
            entryY[fromFace] = fromY;
            scoreG[fromFace] = FixMath.F64.Zero;
            var dist = FixMath.F64.Sqrt((toX - fromX) * (toX - fromX) + (toY - fromY) * (toY - fromY));
            scoreH[fromFace] = dist;
            scoreF[fromFace] = dist;

            Edge innerEdge;
            Face neighbourFace;
            var f = FixMath.F64.Zero;
            var g = FixMath.F64.Zero;
            var h = FixMath.F64.Zero;
            var fromPoint = FixMath.F64Vec2.Zero;
            var entryPoint = FixMath.F64Vec2.Zero;
            var distancePoint = FixMath.F64Vec2.Zero;
            var fillDatas = false;
            while (true)
            {
                // no path found
                if (sortedOpenedFaces.Count == 0)
                {
                    //Debug.trace("AStar no path found");
                    curFace = null;
                    break;
                }  // we reached the target face  

                curFace = sortedOpenedFaces[0];
                sortedOpenedFaces.RemoveAt(0);
                if (curFace == toFace)
                    break;
                // we continue the search  
                iterEdge.set_fromFace(curFace);
                while ((innerEdge = iterEdge.next()) != null)
                {
                    if (innerEdge._isConstrained)
                        continue;
                    neighbourFace = innerEdge.get_rightFace();
                    if (!closedFaces.TryGetValue(neighbourFace, out var closed) || !closed)
                    {
                        if (curFace != fromFace && _radius > 0 && !isWalkableByRadius(entryEdges[curFace], curFace, innerEdge))
                        {
                            //                            Debug.trace("- NOT WALKABLE -");
                            //                            Debug.trace( "from ", hxDaedalusEdge(__entryEdges[__curFace]).originVertex.id, hxDaedalusEdge(__entryEdges[__curFace]).destinationVertex.id );
                            //                            Debug.trace( "to", innerEdge.originVertex.id, innerEdge.destinationVertex.id );
                            //                            Debug.trace("----------------");
                            continue;
                        }

                        fromPoint.X = entryX[curFace];
                        fromPoint.Y = entryY[curFace];
                        entryPoint.X = (innerEdge.get_originVertex()._pos.X + innerEdge.get_destinationVertex()._pos.X) / 2;
                        entryPoint.Y = (innerEdge.get_originVertex()._pos.Y + innerEdge.get_destinationVertex()._pos.Y) / 2;
                        distancePoint.X = entryPoint.X - toX;
                        distancePoint.Y = entryPoint.Y - toY;
                        h = FixMath.F64Vec2.LengthFast(distancePoint);
                        distancePoint.X = fromPoint.X - entryPoint.X;
                        distancePoint.Y = fromPoint.Y - entryPoint.Y;
                        g = scoreG[curFace] + FixMath.F64Vec2.LengthFast(distancePoint);
                        f = h + g;
                        fillDatas = false;
                        if (!openedFaces.TryGetValue(neighbourFace, out var open) || !open)
                        {
                            sortedOpenedFaces.Add(neighbourFace);
                            openedFaces[neighbourFace] = true;
                            fillDatas = true;
                        }
                        else if (scoreF[neighbourFace] > f)
                        {
                            fillDatas = true;
                        }
                        if (fillDatas)
                        {
                            entryEdges[neighbourFace] = innerEdge;
                            entryX[neighbourFace] = entryPoint.X;
                            entryY[neighbourFace] = entryPoint.Y;
                            scoreF[neighbourFace] = f;
                            scoreG[neighbourFace] = g;
                            scoreH[neighbourFace] = h;
                            predecessor[neighbourFace] = curFace;
                        }
                    }
                }  //  

                openedFaces[curFace] = false;
                closedFaces[curFace] = true;
                sortedOpenedFaces.Sort(sortingFaces);
            }  // if we didn't find a path  



            if (curFace == null)
                return;  // else we build the path  ;



            resultListFaces.Add(curFace);
            //curFace.colorDebug = 0x0000FF;
            while (curFace != fromFace)
            {
                resultListEdges.Insert(0, entryEdges[curFace]);
                //entryEdges[__curFace].colorDebug = 0xFFFF00;
                //entryEdges[__curFace].oppositeEdge.colorDebug = 0xFFFF00;
                curFace = predecessor[curFace];
                //curFace.colorDebug = 0x0000FF;
                resultListFaces.Insert(0, curFace);
            }
        }

        // faces with low distance value are at the end of the array
        int sortingFaces(Face a, Face b)
        {
            return scoreF[a].CompareTo(scoreG[b]);
        }

        bool isWalkableByRadius(Edge fromEdge, Face throughFace, Edge toEdge) 
        {
            Vertex vA = null;  // the vertex on fromEdge not on toEdge  
            Vertex vB = null;  // the vertex on toEdge not on fromEdge  
            Vertex vC = null;  // the common vertex of the 2 edges (pivot)  

            // we identify the points
            if (fromEdge.get_originVertex() == toEdge.get_originVertex())
            {
                vA = fromEdge.get_destinationVertex();
                vB = toEdge.get_destinationVertex();
                vC = fromEdge.get_originVertex();
            }
            else if (fromEdge.get_destinationVertex() == toEdge.get_destinationVertex())
            {
                vA = fromEdge.get_originVertex();
                vB = toEdge.get_originVertex();
                vC = fromEdge.get_destinationVertex();
            }
            else if (fromEdge.get_originVertex() == toEdge.get_destinationVertex())
            {
                vA = fromEdge.get_destinationVertex();
                vB = toEdge.get_originVertex();
                vC = fromEdge.get_originVertex();
            }
            else if (fromEdge.get_destinationVertex() == toEdge.get_originVertex())
            {
                vA = fromEdge.get_originVertex();
                vB = toEdge.get_destinationVertex();
                vC = fromEdge.get_destinationVertex();
            }

            FixMath.F64 dot;
            bool result;
            FixMath.F64 distSquared;

            // if we have a right or obtuse angle on CAB
            dot = (vC._pos.X - vA._pos.X) * (vB._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vB._pos.Y - vA._pos.Y);
            if (dot <= 0)
            {
                // we compare length of AC with radius
                distSquared = (vC._pos.X - vA._pos.X) * (vC._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vC._pos.Y - vA._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }  // if we have a right or obtuse angle on CBA  



            dot = (vC._pos.X - vB._pos.X) * (vA._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vA._pos.Y - vB._pos.Y);
            if (dot <= 0)
            {
                // we compare length of BC with radius
                distSquared = (vC._pos.X - vB._pos.X) * (vC._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vC._pos.Y - vB._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }  // we identify the adjacent edge (facing pivot vertex)  



            Edge adjEdge;
            if (throughFace._edge != fromEdge && throughFace._edge._oppositeEdge != fromEdge && throughFace._edge != toEdge && throughFace._edge._oppositeEdge != toEdge)
            {
                adjEdge = throughFace._edge;
            }
            else if (throughFace._edge.get_nextLeftEdge() != fromEdge && throughFace._edge.get_nextLeftEdge()._oppositeEdge != fromEdge && throughFace._edge.get_nextLeftEdge() != toEdge && throughFace._edge.get_nextLeftEdge()._oppositeEdge != toEdge)
            {
                adjEdge = throughFace._edge.get_nextLeftEdge();
            }
            else
            {
                adjEdge = throughFace._edge.get_prevLeftEdge();
            }
            // if the adjacent edge is constrained, we check the distance of orthognaly projected
            if (adjEdge._isConstrained)
            {
                var proj = new FixMath.F64Vec2(vC._pos.X, vC._pos.Y);
                Geom2D.projectOrthogonaly(ref proj, adjEdge);
                distSquared = (proj.X - vC._pos.X) * (proj.X - vC._pos.X) + (proj.Y - vC._pos.Y) * (proj.Y - vC._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            // if the adjacent is not constrained
            else
            {
                var distSquaredA = (vC._pos.X - vA._pos.X) * (vC._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vC._pos.Y - vA._pos.Y);
                var distSquaredB = (vC._pos.X - vB._pos.X) * (vC._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vC._pos.Y - vB._pos.Y);
                if (distSquaredA < diameterSquared || distSquaredB < diameterSquared)
                {
                    return false;
                }
                else
                {
                    var vFaceToCheck = new List<Face>();
                    var vFaceIsFromEdge = new List<Edge>();
                    var facesDone = new Dictionary<Face, bool>();
                    vFaceIsFromEdge.Add(adjEdge);
                    if (adjEdge.get_leftFace() == throughFace)
                    {
                        vFaceToCheck.Add(adjEdge.get_rightFace());
                        facesDone[adjEdge.get_rightFace()] = true;
                    }
                    else
                    {
                        vFaceToCheck.Add(adjEdge.get_leftFace());
                        facesDone[adjEdge.get_leftFace()] = true;
                    }

                    Face currFace;
                    Edge faceFromEdge;
                    Edge currEdgeA;
                    Face nextFaceA;
                    Edge currEdgeB;
                    Face nextFaceB;
                    while (vFaceToCheck.Count > 0)
                    {
                        currFace = vFaceToCheck[0];
                        vFaceToCheck.RemoveAt(0);

                        faceFromEdge = vFaceIsFromEdge[0];
                        vFaceIsFromEdge.RemoveAt(0);

                        if (currFace._edge == faceFromEdge || currFace._edge == faceFromEdge._oppositeEdge)
                        {
                            // we identify the 2 edges to evaluate
                            currEdgeA = currFace._edge.get_nextLeftEdge();
                            currEdgeB = currFace._edge.get_nextLeftEdge().get_nextLeftEdge();
                        }
                        else if (currFace._edge.get_nextLeftEdge() == faceFromEdge || currFace._edge.get_nextLeftEdge() == faceFromEdge._oppositeEdge)
                        {
                            // we identify the faces related to the 2 edges
                            currEdgeA = currFace._edge;
                            currEdgeB = currFace._edge.get_nextLeftEdge().get_nextLeftEdge();
                        }
                        else
                        {
                            currEdgeA = currFace._edge;
                            currEdgeB = currFace._edge.get_nextLeftEdge();
                        }

                        if (currEdgeA.get_leftFace() == currFace)
                        {
                            nextFaceA = currEdgeA.get_rightFace();
                        }
                        else
                        {
                            nextFaceA = currEdgeA.get_leftFace();
                        }
                        if (currEdgeB.get_leftFace() == currFace)
                        {
                            nextFaceB = currEdgeB.get_rightFace();
                        }
                        else
                        {
                            nextFaceB = currEdgeB.get_leftFace();
                        }
                        // we check if the next face is not already in pipe
                        // and if the edge A is close to pivot vertex
                        var nextFaceA_not_done = !facesDone.TryGetValue(nextFaceA, out var doneA) || !doneA;
                        if (nextFaceA_not_done && Geom2D.distanceSquaredVertexToEdge(vC, currEdgeA) < diameterSquared)
                        {
                            // if the edge is constrained
                            if (currEdgeA._isConstrained)
                            {
                                // so it is not walkable
                                return false;
                            }
                            else
                            {
                                // if the edge is not constrained, we continue the search
                                vFaceToCheck.Add(nextFaceA);
                                vFaceIsFromEdge.Add(currEdgeA);
                                facesDone[nextFaceA] = true;
                            }
                        }  // and if the edge B is close to pivot vertex    // we check if the next face is not already in pipe  

                        var nextFaceB_not_done = !facesDone.TryGetValue(nextFaceB, out var doneB) || !doneB;
                        if (nextFaceB_not_done && Geom2D.distanceSquaredVertexToEdge(vC, currEdgeB) < diameterSquared)
                        {
                            // if the edge is constrained
                            if (currEdgeB._isConstrained)
                            {
                                // so it is not walkable
                                return false;
                            }
                            else
                            {
                                // if the edge is not constrained, we continue the search
                                vFaceToCheck.Add(nextFaceB);
                                vFaceIsFromEdge.Add(currEdgeB);
                                facesDone[nextFaceB] = true;
                            }
                        }
                    }  // if we didn't previously meet a constrained edge  

                    return true;
                }
            }

            return true;
        }
    }
}
