
using System.Collections.Generic;
using System.Linq;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Triangulation.Math;
using Pathfinding.Util;

namespace Pathfinding.Triangulation.Data
{
    public class Mesh
    {
        static Mesh()
        {
            Mesh.INC = 0;
        }

        public Mesh(FixMath.F64 width, FixMath.F64 height)
        {
            _id = INC;
            INC++;

            _width = width;
            _height = height;
            _clipping = true;

            _vertices = new List<Vertex>();
            _edges = new List<Edge>();
            _faces = new List<Face>();
            _constraintShapes = new List<ConstraintShape>();
            _objects = new List<Object>();

            __centerVertex = null;
            __edgesToCheck = new List<Edge>();
        }

        public static int INC;

        public int _id;

        public FixMath.F64 _width;

        public FixMath.F64 _height;

        public bool _clipping;

        public List<Vertex> _vertices;

        public List<Edge> _edges;

        public List<Face> _faces;

        public List<ConstraintShape> _constraintShapes;

        public List<Object> _objects;

        // keep references of center vertex and bounding edges when split, useful to restore edges as Delaunay
        public Vertex __centerVertex;
        public List<Edge> __edgesToCheck;

        public FixMath.F64 get_height()
        {
            return this._height;
        }


        public FixMath.F64 get_width()
        {
            return this._width;
        }


        public bool get_clipping()
        {
            return this._clipping;
        }

        public bool set_clipping(bool @value)
        {
            this._clipping = @value;
            return @value;
        }

        public int get_id()
        {
            return this._id;
        }

        public void dispose()
        {
            while ((_vertices.Count > 0))
            {
                var index = _vertices.Count - 1;
                _vertices[index].dispose();
                _vertices.RemoveAt(index);
            }
            _vertices = null;

            while ((_edges.Count > 0))
            {
                var index = _edges.Count - 1;
                _edges[index].dispose();
                _edges.RemoveAt(index);
            }
            _edges = null;

            while ((_faces.Count > 0))
            {
                var index = _faces.Count - 1;
                _faces[index].dispose();
                _faces.RemoveAt(index);
            }
            _faces = null;

            while ((_constraintShapes.Count > 0))
            {
                var index = _constraintShapes.Count - 1;
                _constraintShapes[index].dispose();
                _constraintShapes.RemoveAt(index);
            }
            _constraintShapes = null;

            while ((_objects.Count > 0))
            {
                var index = _objects.Count - 1;
                _objects[index].dispose();
                _objects.RemoveAt(index);
            }

            this._objects = null;
            this.__edgesToCheck = null;
            this.__centerVertex = null;
        }


        public List<ConstraintShape> get_constraintShapes()
        {
            return this._constraintShapes;
        }


        public void buildFromRecord(string rec)
        {
            var positions = rec.Split(";");
            var i = 0;
            while (i < positions.Length)
            {
                insertConstraintSegment(
                    FixMath.F64.FromFloat(float.Parse(positions[i])),
                    FixMath.F64.FromFloat(float.Parse(positions[i + 1])),
                    FixMath.F64.FromFloat(float.Parse(positions[i + 2])),
                    FixMath.F64.FromFloat(float.Parse(positions[i + 3])));
                i += 4;
            }
        }


        public void insertObject(Object object_)
        {
            if (object_._constraintShape != null)
                deleteObject(object_);

            var shape = new ConstraintShape();
            ConstraintSegment segment;
            var coordinates = object_._coordinates;
            object_.updateMatrixFromValues();

            var m = object_._matrix;

            FixMath.F64 x1;
            FixMath.F64 y1;
            FixMath.F64 x2;
            FixMath.F64 y2;
            FixMath.F64 transfx1;
            FixMath.F64 transfy1;
            FixMath.F64 transfx2;
            FixMath.F64 transfy2;

            var i = 0;
            while (i < coordinates.Count)
            {
                x1 = coordinates[i];
                y1 = coordinates[i + 1];
                x2 = coordinates[i + 2];
                y2 = coordinates[i + 3];
                transfx1 = m.transformX(x1, y1);
                transfy1 = m.transformY(x1, y1);
                transfx2 = m.transformX(x2, y2);
                transfy2 = m.transformY(x2, y2);

                segment = insertConstraintSegment(transfx1, transfy1, transfx2, transfy2);
                if (segment != null)
                {
                    segment.fromShape = shape;
                    shape.segments.Add(segment);
                }
                i += 4;
            }

            _constraintShapes.Add(shape);
            object_._constraintShape = shape;

            if (!__objectsUpdateInProgress)
            {
                _objects.Add(object_);
            }
        }


        public void deleteObject(Object object_)
        {
            if (object_._constraintShape == null)
                return;


            deleteConstraintShape(object_._constraintShape);
            object_._constraintShape = null;

            if (!__objectsUpdateInProgress)
            {
                _objects.Remove(object_);
            }
        }


        public bool __objectsUpdateInProgress = false;

        public virtual void updateObjects()
        {
            __objectsUpdateInProgress = true;
            for (var i = 0; i < _objects.Count; ++i)
            {
                if (_objects[i]._hasChanged)
                {
                    deleteObject(_objects[i]);
                    insertObject(_objects[i]);
                    _objects[i]._hasChanged = false;
                }
            }
            __objectsUpdateInProgress = false;
        }


        // insert a new collection of constrained edges.
        // Coordinates parameter is a list with form [x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, ....]
        // where each 4-uple sequence (xi, yi, xi+1, yi+1) is a constraint segment (with i % 4 == 0)
        // and where each couple sequence (xi, yi) is a point.
        // Segments are not necessary connected.
        // Segments can overlap (then they will be automaticaly subdivided).
        public ConstraintShape insertConstraintShape(List<FixMath.F64> coordinates)
        {
            var shape = new ConstraintShape();
            ConstraintSegment segment = null;

            var i = 0;
            while (i < coordinates.Count)
            {
                segment = insertConstraintSegment(coordinates[i], coordinates[i + 1], coordinates[i + 2], coordinates[i + 3]);
                if (segment != null)
                {
                    segment.fromShape = shape;
                    shape.segments.Add(segment);
                }
                i += 4;
            }

            _constraintShapes.Add(shape);

            return shape;
        }


        public void deleteConstraintShape(ConstraintShape shape)
        {
            for (var i = 0; i < shape.segments.Count; ++i)
                deleteConstraintSegment(shape.segments[i]);
            shape.dispose();
            _constraintShapes.Remove(shape);
        }


        public ConstraintSegment insertConstraintSegment(FixMath.F64 x1, FixMath.F64 y1, FixMath.F64 x2, FixMath.F64 y2)
        {
            //Debug.trace("insertConstraintSegment");

            /* point positions relative to bounds
            1 | 2 | 3
            ------------
            8 | 0 | 4
            ------------
            7 | 6 | 5
            */
            var p1pos = findPositionFromBounds(x1, y1);
            var p2pos = findPositionFromBounds(x2, y2);

            var newX1 = x1;
            var newY1 = y1;
            var newX2 = x2;
            var newY2 = y2;
            // need clipping if activated and if one end point is outside bounds
            if (_clipping && (p1pos != 0 || p2pos != 0))
            {
                var intersectPoint = FixMath.F64Vec2.Zero;

                // if both end points are outside bounds
                if (p1pos != 0 && p2pos != 0)
                {
                    // if both end points are on same side
                    if ((x1 <= 0 && x2 <= 0) || (x1 >= _width && x2 >= _width) || (y1 <= 0 && y2 <= 0) || (y1 >= _height && y2 >= _height))
                        return null;  // if end points are in separated left and right areas  ;



                    if ((p1pos == 8 && p2pos == 4) || (p1pos == 4 && p2pos == 8))
                    {
                        // intersection with left bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint);
                        newX1 = intersectPoint.X;
                        newY1 = intersectPoint.Y;
                        // intersection with right bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint);
                        newX2 = intersectPoint.X;
                        newY2 = intersectPoint.Y;
                    }
                    // if end points are in separated top and bottom areas
                    else if ((p1pos == 2 && p2pos == 6) || (p1pos == 6 && p2pos == 2))
                    {
                        // intersection with top bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint);
                        newX1 = intersectPoint.X;
                        newY1 = intersectPoint.Y;
                        // intersection with bottom bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint);
                        newX2 = intersectPoint.X;
                        newY2 = intersectPoint.Y;
                    }
                    // if ends points are apart of the top-left corner
                    else if ((p1pos == 2 && p2pos == 8) || (p1pos == 8 && p2pos == 2))
                    {
                        // check if intersection with top bound
                        if (Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint))
                        {
                            newX1 = intersectPoint.X;
                            newY1 = intersectPoint.Y;

                            // must have intersection with left bound
                            Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint);
                            newX2 = intersectPoint.X;
                            newY2 = intersectPoint.Y;
                        }
                        else
                            return null;
                    }
                    // if ends points are apart of the top-right corner
                    else if ((p1pos == 2 && p2pos == 4) || (p1pos == 4 && p2pos == 2))
                    {
                        // check if intersection with top bound
                        if (Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint))
                        {
                            newX1 = intersectPoint.X;
                            newY1 = intersectPoint.Y;

                            // must have intersection with right bound
                            Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint);
                            newX2 = intersectPoint.X;
                            newY2 = intersectPoint.Y;
                        }
                        else
                            return null;
                    }
                    // if ends points are apart of the bottom-right corner
                    else if ((p1pos == 6 && p2pos == 4) || (p1pos == 4 && p2pos == 6))
                    {
                        // check if intersection with bottom bound
                        if (Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint))
                        {
                            newX1 = intersectPoint.X;
                            newY1 = intersectPoint.Y;

                            // must have intersection with right bound
                            Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint);
                            newX2 = intersectPoint.X;
                            newY2 = intersectPoint.Y;
                        }
                        else
                            return null;
                    }
                    // if ends points are apart of the bottom-left corner
                    else if ((p1pos == 8 && p2pos == 6) || (p1pos == 6 && p2pos == 8))
                    {
                        // check if intersection with bottom bound
                        if (Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint))
                        {
                            newX1 = intersectPoint.X;
                            newY1 = intersectPoint.Y;

                            // must have intersection with left bound
                            Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint);
                            newX2 = intersectPoint.X;
                            newY2 = intersectPoint.Y;
                        }
                        else
                            return null;
                    }
                    // other cases (could be optimized)
                    else
                    {
                        var firstDone = false;
                        var secondDone = false;
                        // check top bound
                        if (Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint))
                        {
                            newX1 = intersectPoint.X;
                            newY1 = intersectPoint.Y;
                            firstDone = true;
                        }  // check right bound  

                        if (Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint))
                        {
                            if (!firstDone)
                            {
                                newX1 = intersectPoint.X;
                                newY1 = intersectPoint.Y;
                                firstDone = true;
                            }
                            else
                            {
                                newX2 = intersectPoint.X;
                                newY2 = intersectPoint.Y;
                                secondDone = true;
                            }
                        }  // check bottom bound  

                        if (!secondDone && Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint))
                        {
                            if (!firstDone)
                            {
                                newX1 = intersectPoint.X;
                                newY1 = intersectPoint.Y;
                                firstDone = true;
                            }
                            else
                            {
                                newX2 = intersectPoint.X;
                                newY2 = intersectPoint.Y;
                                secondDone = true;
                            }
                        }  // check left bound  

                        if (!secondDone && Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint))
                        {
                            newX2 = intersectPoint.X;
                            newY2 = intersectPoint.Y;
                        }

                        if (!firstDone)
                            return null;
                    }
                }
                // one end point of segment is outside bounds and one is inside
                else
                {
                    // if one point is outside top
                    if (p1pos == 2 || p2pos == 2)
                    {
                        // intersection with top bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint);
                    }
                    // if one point is outside right
                    else if (p1pos == 4 || p2pos == 4)
                    {
                        // intersection with right bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint);
                    }
                    // if one point is outside bottom
                    else if (p1pos == 6 || p2pos == 6)
                    {
                        // intersection with bottom bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint);
                    }
                    // if one point is outside left
                    else if (p1pos == 8 || p2pos == 8)
                    {
                        // intersection with left bound
                        Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint);
                    }
                    // other cases (could be optimized)
                    else
                    {
                        // check top bound
                        if (!Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, _width, FixMath.F64.Zero, out intersectPoint))
                        {
                            // check right bound
                            if (!Geom2D.intersections2segments(x1, y1, x2, y2, _width, FixMath.F64.Zero, _width, _height, out intersectPoint))
                            {
                                // check bottom bound
                                if (!Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, _height, _width, _height, out intersectPoint))
                                {
                                    // check left bound
                                    Geom2D.intersections2segments(x1, y1, x2, y2, FixMath.F64.Zero, FixMath.F64.Zero, FixMath.F64.Zero, _height, out intersectPoint);
                                }
                            }
                        }
                    }

                    if (p1pos == 0)
                    {
                        newX1 = x1;
                        newY1 = y1;
                    }
                    else
                    {
                        newX1 = x2;
                        newY1 = y2;
                    }
                    newX2 = intersectPoint.X;
                    newY2 = intersectPoint.Y;
                }
            }  // we check the vertices insertions  

            var vertexDown = insertVertex(newX1, newY1);
            if (vertexDown == null)
                return null;
            var vertexUp = insertVertex(newX2, newY2);
            if (vertexUp == null)
                return null;
            if (vertexDown == vertexUp)
                return null;
            // useful    //Debug.trace("vertices " + vertexDown.id + " " + vertexUp.id)  
            var iterVertexToOutEdges = new FromVertexToOutgoingEdges();
            Vertex currVertex;
            Edge currEdge;
            var i = 0;

            // the new constraint segment
            var segment = new ConstraintSegment();

            var tempEdgeDownUp = new Edge();
            var tempSdgeUpDown = new Edge();
            tempEdgeDownUp.setDatas(vertexDown, tempSdgeUpDown, null, null, true, true);
            tempSdgeUpDown.setDatas(vertexUp, tempEdgeDownUp, null, null, true, true);

            var intersectedEdges = new List<Edge>();
            var leftBoundingEdges = new List<Edge>();
            var rightBoundingEdges = new List<Edge>();

            Intersection currObjet = null ;
            var pIntersect = FixMath.F64Vec2.Zero;
            Edge edgeLeft;
            Edge newEdgeDownUp;
            Edge newEdgeUpDown;
            var done = false;
            currVertex = vertexDown;
            currObjet = Intersection.EVertex(currVertex);
            while (true)
            {
                done = false;

                if (currObjet is Intersection_EVertex v)
                {
                    ///////////////////////////
                    //Debug.trace("case vertex");
                    currVertex = v.vertex;
                    iterVertexToOutEdges.set_fromVertex(currVertex);
                    while ((currEdge = iterVertexToOutEdges.next()) != null)
                    {
                        // if we meet directly the end vertex
                        if (currEdge.get_destinationVertex() == vertexUp)
                        {
                            //Debug.trace("we met the end vertex");
                            if (!currEdge._isConstrained)
                            {
                                currEdge._isConstrained = true;
                                currEdge._oppositeEdge._isConstrained = true;
                            }
                            currEdge.addFromConstraintSegment(segment);
                            currEdge._oppositeEdge.fromConstraintSegments = currEdge.fromConstraintSegments;
                            vertexDown.addFromConstraintSegment(segment);
                            vertexUp.addFromConstraintSegment(segment);
                            segment.addEdge(currEdge);
                            return segment;
                        }  // if we meet a vertex  

                        if (Geom2D.distanceSquaredVertexToEdge(currEdge.get_destinationVertex(), tempEdgeDownUp) <= Constants.EPSILON_SQUARED)
                        {
                            //Debug.trace("we met a vertex");
                            if (!currEdge._isConstrained)
                            {
                                //Debug.trace("edge is not constrained");
                                currEdge._isConstrained = true;
                                currEdge._oppositeEdge._isConstrained = true;
                            }
                            currEdge.addFromConstraintSegment(segment);
                            currEdge._oppositeEdge.fromConstraintSegments = currEdge.fromConstraintSegments;
                            vertexDown.addFromConstraintSegment(segment);
                            segment.addEdge(currEdge);
                            vertexDown = currEdge.get_destinationVertex();
                            tempEdgeDownUp.set_originVertex(vertexDown);
                            currObjet = Intersection.EVertex(vertexDown);
                            done = true;
                            break;
                        }
                    }

                    if (done)
                        continue;

                    iterVertexToOutEdges.set_fromVertex(currVertex);
                    while ((currEdge = iterVertexToOutEdges.next()) != null)
                    {
                        currEdge = currEdge.get_nextLeftEdge();
                        if (Geom2D.intersections2edges(currEdge, tempEdgeDownUp, out pIntersect))
                        {
                            if (currEdge._isConstrained)
                            {
                                vertexDown = splitEdge(currEdge, pIntersect.X, pIntersect.Y);
                                iterVertexToOutEdges.set_fromVertex(currVertex);
                                while ((currEdge = iterVertexToOutEdges.next()) != null)
                                {
                                    if (currEdge.get_destinationVertex() == vertexDown)
                                    {
                                        currEdge._isConstrained = true;
                                        currEdge._oppositeEdge._isConstrained = true;
                                        currEdge.addFromConstraintSegment(segment);
                                        currEdge._oppositeEdge.fromConstraintSegments = currEdge.fromConstraintSegments;
                                        segment.addEdge(currEdge);
                                        break;
                                    }
                                }
                                currVertex.addFromConstraintSegment(segment);
                                tempEdgeDownUp.set_originVertex(vertexDown);
                                currObjet = Intersection.EVertex(vertexDown);
                            }
                            else
                            {
                                intersectedEdges.Add(currEdge);
                                leftBoundingEdges.Insert(0, currEdge.get_nextLeftEdge());
                                rightBoundingEdges.Add(currEdge.get_prevLeftEdge());
                                currEdge = currEdge._oppositeEdge;  // we keep the edge from left to right  
                                currObjet = Intersection.EEdge(currEdge);
                            }
                            break;
                        }
                    }
                }
                else if (currObjet is Intersection_EEdge e)
                {
                    currEdge = e.edge;
                    edgeLeft = currEdge.get_nextLeftEdge();
                    if (edgeLeft.get_destinationVertex() == vertexUp)
                    {
                        //Debug.trace("end point reached");
                        leftBoundingEdges.Insert(0, edgeLeft.get_nextLeftEdge());
                        rightBoundingEdges.Add(edgeLeft);

                        newEdgeDownUp = new Edge();
                        newEdgeUpDown = new Edge();
                        newEdgeDownUp.setDatas(vertexDown, newEdgeUpDown, null, null, true, true);
                        newEdgeUpDown.setDatas(vertexUp, newEdgeDownUp, null, null, true, true);
                        leftBoundingEdges.Add(newEdgeDownUp);
                        rightBoundingEdges.Add(newEdgeUpDown);
                        insertNewConstrainedEdge(segment, newEdgeDownUp, intersectedEdges, leftBoundingEdges, rightBoundingEdges);

                        return segment;
                    }
                    else if (Geom2D.distanceSquaredVertexToEdge(edgeLeft.get_destinationVertex(), tempEdgeDownUp) <= Constants.EPSILON_SQUARED)
                    {
                        //Debug.trace("we met a vertex");
                        leftBoundingEdges.Insert(0, edgeLeft.get_nextLeftEdge());
                        rightBoundingEdges.Add(edgeLeft);

                        newEdgeDownUp = new Edge();
                        newEdgeUpDown = new Edge();
                        newEdgeDownUp.setDatas(vertexDown, newEdgeUpDown, null, null, true, true);
                        newEdgeUpDown.setDatas(edgeLeft.get_destinationVertex(), newEdgeDownUp, null, null, true, true);
                        leftBoundingEdges.Add(newEdgeDownUp);
                        rightBoundingEdges.Add(newEdgeUpDown);
                        insertNewConstrainedEdge(segment, newEdgeDownUp, intersectedEdges, leftBoundingEdges, rightBoundingEdges);

                        intersectedEdges.Clear();
                        leftBoundingEdges.Clear();
                        rightBoundingEdges.Clear();

                        vertexDown = edgeLeft.get_destinationVertex();
                        tempEdgeDownUp.set_originVertex(vertexDown);
                        currObjet = Intersection.EVertex(vertexDown);
                    }
                    else
                    {
                        if (Geom2D.intersections2edges(edgeLeft, tempEdgeDownUp, out pIntersect))
                        {
                            //Debug.trace("1st left edge intersected");
                            if (edgeLeft._isConstrained)
                            {
                                //Debug.trace("edge is constrained");
                                currVertex = splitEdge(edgeLeft, pIntersect.X, pIntersect.Y);

                                iterVertexToOutEdges.set_fromVertex(currVertex);
                                while ((currEdge = iterVertexToOutEdges.next()) != null)
                                {
                                    if (currEdge.get_destinationVertex() == leftBoundingEdges[0].get_originVertex())
                                    {
                                        leftBoundingEdges.Insert(0, currEdge);
                                    }
                                    if (currEdge.get_destinationVertex() == rightBoundingEdges[rightBoundingEdges.Count - 1].get_destinationVertex())
                                    {
                                        rightBoundingEdges.Add(currEdge._oppositeEdge);
                                    }
                                }

                                newEdgeDownUp = new Edge();
                                newEdgeUpDown = new Edge();
                                newEdgeDownUp.setDatas(vertexDown, newEdgeUpDown, null, null, true, true);
                                newEdgeUpDown.setDatas(currVertex, newEdgeDownUp, null, null, true, true);
                                leftBoundingEdges.Add(newEdgeDownUp);
                                rightBoundingEdges.Add(newEdgeUpDown);
                                insertNewConstrainedEdge(segment, newEdgeDownUp, intersectedEdges, leftBoundingEdges, rightBoundingEdges);

                                intersectedEdges.Clear();
                                leftBoundingEdges.Clear();
                                rightBoundingEdges.Clear();
                                vertexDown = currVertex;
                                tempEdgeDownUp.set_originVertex(vertexDown);
                                currObjet = Intersection.EVertex(vertexDown);
                            }
                            else
                            {
                                //Debug.trace("edge is not constrained");
                                intersectedEdges.Add(edgeLeft);
                                leftBoundingEdges.Insert(0, edgeLeft.get_nextLeftEdge());
                                currEdge = edgeLeft._oppositeEdge;  // we keep the edge from left to right  
                                currObjet = Intersection.EEdge(currEdge);
                            }
                        }
                        else
                        {
                            //Debug.trace("2nd left edge intersected");
                            edgeLeft = edgeLeft.get_nextLeftEdge();
                            Geom2D.intersections2edges(edgeLeft, tempEdgeDownUp, out pIntersect);
                            if (edgeLeft._isConstrained)
                            {
                                //Debug.trace("edge is constrained");
                                currVertex = splitEdge(edgeLeft, pIntersect.X, pIntersect.Y);

                                iterVertexToOutEdges.set_fromVertex(currVertex);
                                while ((currEdge = iterVertexToOutEdges.next()) != null)
                                {
                                    if (currEdge.get_destinationVertex() == leftBoundingEdges[0].get_originVertex())
                                    {
                                        leftBoundingEdges.Insert(0, currEdge);
                                    }
                                    if (currEdge.get_destinationVertex() == rightBoundingEdges[rightBoundingEdges.Count - 1].get_destinationVertex())
                                    {
                                        rightBoundingEdges.Add(currEdge._oppositeEdge);
                                    }
                                }

                                newEdgeDownUp = new Edge();
                                newEdgeUpDown = new Edge();
                                newEdgeDownUp.setDatas(vertexDown, newEdgeUpDown, null, null, true, true);
                                newEdgeUpDown.setDatas(currVertex, newEdgeDownUp, null, null, true, true);
                                leftBoundingEdges.Add(newEdgeDownUp);
                                rightBoundingEdges.Add(newEdgeUpDown);
                                insertNewConstrainedEdge(segment, newEdgeDownUp, intersectedEdges, leftBoundingEdges, rightBoundingEdges);

                                intersectedEdges.Clear();
                                leftBoundingEdges.Clear();
                                rightBoundingEdges.Clear();
                                vertexDown = currVertex;
                                tempEdgeDownUp.set_originVertex(vertexDown);
                                currObjet = Intersection.EVertex(vertexDown);
                            }
                            else
                            {
                                //Debug.trace("edge is not constrained");
                                intersectedEdges.Add(edgeLeft);
                                rightBoundingEdges.Add(edgeLeft.get_prevLeftEdge());
                                currEdge = edgeLeft._oppositeEdge;  // we keep the edge from left to right  
                                currObjet = Intersection.EEdge(currEdge);
                            }
                        }
                    }
                }
            }

            return segment;
        }


        public void insertNewConstrainedEdge(ConstraintSegment fromSegment, Edge edgeDownUp, List<Edge> intersectedEdges, List<Edge> leftBoundingEdges, List<Edge> rightBoundingEdges)
        {
            this._edges.Add(edgeDownUp);
            this._edges.Add(edgeDownUp.get_oppositeEdge());
            edgeDownUp.addFromConstraintSegment(fromSegment);
            edgeDownUp.get_oppositeEdge().fromConstraintSegments = edgeDownUp.fromConstraintSegments;
            fromSegment.addEdge(edgeDownUp);
            edgeDownUp.get_originVertex().addFromConstraintSegment(fromSegment);
            edgeDownUp.get_destinationVertex().addFromConstraintSegment(fromSegment);
            this.untriangulate(intersectedEdges);
            this.triangulate(leftBoundingEdges, true);
            this.triangulate(rightBoundingEdges, true);
        }


        public void deleteConstraintSegment(ConstraintSegment segment)
        {
            var vertexToDelete = new List<Vertex>();
            Edge edge = null;
            Vertex vertex = null;

            {
                int _g = 0;
                int _g1 = segment.get_edges().Count;
                while ((_g < _g1))
                {
                    int i1 = _g++;
                    edge = (segment.get_edges()[i1]);
                    edge.removeFromConstraintSegment(segment);
                    if ((edge.fromConstraintSegments.Count == 0))
                    {
                        edge.set_isConstrained(false);
                        edge.get_oppositeEdge().set_isConstrained(false);
                    }

                    vertex = edge.get_originVertex();
                    vertex.removeFromConstraintSegment(segment);
                    vertexToDelete.Add(vertex);
                }

            }

            vertex = edge.get_destinationVertex();
            vertex.removeFromConstraintSegment(segment);
            vertexToDelete.Add(vertex);
            {
                int _g2 = 0;
                int _g3 = vertexToDelete.Count;
                while ((_g2 < _g3))
                {
                    int i2 = _g2++;
                    this.deleteVertex(vertexToDelete[i2]);
                }

            }

            segment.dispose();
        }


        public void check()
        {
            int _g = 0;
            int _g1 = this._edges.Count;
            while ((_g < _g1))
            {
                int i = _g++;
                if (this._edges[i].get_nextLeftEdge() == null)
                {
                    return;
                }

            }

        }


        public Vertex insertVertex(FixMath.F64 x, FixMath.F64 y)
        {
            if (x < 0 || y < 0 || x > this._width || y > this._height)
            {
                return null;
            }

            this.__edgesToCheck.Clear();
            var inObject = Geom2D.locatePosition(x, y, this);
            Vertex newVertex = null;
            if (inObject is Intersection_EVertex v)
            {
                newVertex = v.vertex;
            }
            else if (inObject is Intersection_EEdge e)
            {
                newVertex = this.splitEdge(e.edge, x, y);
            }
            else if (inObject is Intersection_EFace f)
            {
                newVertex = this.splitFace(f.face, x, y);
            }

            this.restoreAsDelaunay();
            return newVertex;
        }


        public Edge flipEdge(Edge edge)
        {
            var eBot_Top = edge;
            var eTop_Bot = edge.get_oppositeEdge();
            var eLeft_Right = new Edge();
            var eRight_Left = new Edge();
            var eTop_Left = eBot_Top.get_nextLeftEdge();
            var eLeft_Bot = eTop_Left.get_nextLeftEdge();
            var eBot_Right = eTop_Bot.get_nextLeftEdge();
            var eRight_Top = eBot_Right.get_nextLeftEdge();
            var vBot = eBot_Top.get_originVertex();
            var vTop = eTop_Bot.get_originVertex();
            var vLeft = eLeft_Bot.get_originVertex();
            var vRight = eRight_Top.get_originVertex();
            var fLeft = eBot_Top.get_leftFace();
            var fRight = eTop_Bot.get_leftFace();
            var fBot = new Face();
            var fTop = new Face();
            // add the new edges
            _edges.Add(eLeft_Right);
            _edges.Add(eRight_Left);

            // add the new faces
            _faces.Add(fTop);
            _faces.Add(fBot);

            // set vertex, edge and face references for the new LEFT_RIGHT and RIGHT-LEFT edges
            eLeft_Right.setDatas(vLeft, eRight_Left, eRight_Top, fTop, edge._isReal, edge._isConstrained);
            eRight_Left.setDatas(vRight, eLeft_Right, eLeft_Bot, fBot, edge._isReal, edge._isConstrained);

            // set edge references for the new TOP and BOTTOM faces
            fTop.setDatas(eLeft_Right);
            fBot.setDatas(eRight_Left);

            // check the edge references of TOP and BOTTOM vertices
            if (vTop._edge == eTop_Bot)
            {
                vTop.setDatas(eTop_Left);
            }
            if (vBot._edge == eBot_Top)
            {
                vBot.setDatas(eBot_Right); // set the new edge and face references for the 4 bouding edges  ;
            }


            eTop_Left._nextLeftEdge = eLeft_Right;
            eTop_Left._leftFace = fTop;
            eLeft_Bot._nextLeftEdge = eBot_Right;
            eLeft_Bot._leftFace = fBot;
            eBot_Right._nextLeftEdge = eRight_Left;
            eBot_Right._leftFace = fBot;
            eRight_Top._nextLeftEdge = eTop_Left;
            eRight_Top._leftFace = fTop;

            // remove the old TOP-BOTTOM and BOTTOM-TOP edges
            eBot_Top.dispose();
            eTop_Bot.dispose();
            _edges.RemoveAt(_edges.IndexOf(eBot_Top));
            _edges.RemoveAt(_edges.IndexOf(eTop_Bot));

            // remove the old LEFT and RIGHT faces
            fLeft.dispose();
            fRight.dispose();
            _faces.RemoveAt(_faces.IndexOf(fLeft));
            _faces.RemoveAt(_faces.IndexOf(fRight));

            return eRight_Left;
        }


        public Vertex splitEdge(Edge edge, FixMath.F64 x, FixMath.F64 y)
        {
            this.__edgesToCheck.Clear();
            Edge eLeft_Right = edge;
            Edge eRight_Left = eLeft_Right.get_oppositeEdge();
            Edge eRight_Top = eLeft_Right.get_nextLeftEdge();
            Edge eTop_Left = eRight_Top.get_nextLeftEdge();
            Edge eLeft_Bot = eRight_Left.get_nextLeftEdge();
            Edge eBot_Right = eLeft_Bot.get_nextLeftEdge();
            Vertex vTop = eTop_Left.get_originVertex();
            Vertex vLeft = eLeft_Right.get_originVertex();
            Vertex vBot = eBot_Right.get_originVertex();
            Vertex vRight = eRight_Left.get_originVertex();
            Face fTop = eLeft_Right.get_leftFace();
            Face fBot = eRight_Left.get_leftFace();
            if ((vLeft._pos.X - x) * (vLeft._pos.X - x) + (vLeft._pos.Y - y) * (vLeft._pos.Y - y) <= Constants.EPSILON_SQUARED)
            {
                return vLeft;
            }

            if ((vRight._pos.X - x) * (vRight._pos.X - x) + (vRight._pos.Y - y) * (vRight._pos.Y - y) <= Constants.EPSILON_SQUARED)
            {
                return vRight;
            }

            Vertex vCenter = new Vertex();
            Edge eTop_Center = new Edge();
            Edge eCenter_Top = new Edge();
            Edge eBot_Center = new Edge();
            Edge eCenter_Bot = new Edge();
            Edge eLeft_Center = new Edge();
            Edge eCenter_Left = new Edge();
            Edge eRight_Center = new Edge();
            Edge eCenter_Right = new Edge();
            Face fTopLeft = new Face();
            Face fBotLeft = new Face();
            Face fBotRight = new Face();
            Face fTopRight = new Face();
            this._vertices.Add(vCenter);
            this._edges.Add(eCenter_Top);
            this._edges.Add(eTop_Center);
            this._edges.Add(eCenter_Left);
            this._edges.Add(eLeft_Center);
            this._edges.Add(eCenter_Bot);
            this._edges.Add(eBot_Center);
            this._edges.Add(eCenter_Right);
            this._edges.Add(eRight_Center);
            this._faces.Add(fTopRight);
            this._faces.Add(fBotRight);
            this._faces.Add(fBotLeft);
            this._faces.Add(fTopLeft);
            // set pos and edge reference for the new CENTER vertex
            vCenter.setDatas((fTop.get_isReal()) ? eCenter_Top : eCenter_Bot);
            vCenter._pos.X = x;
            vCenter._pos.Y = y;
            Geom2D.projectOrthogonaly(ref vCenter._pos, eLeft_Right);

            // set the new vertex, edge and face references for the new 8 center crossing edges
            eCenter_Top.setDatas(vCenter, eTop_Center, eTop_Left, fTopLeft, fTop._isReal);
            eTop_Center.setDatas(vTop, eCenter_Top, eCenter_Right, fTopRight, fTop._isReal);
            eCenter_Left.setDatas(vCenter, eLeft_Center, eLeft_Bot, fBotLeft, edge._isReal, edge._isConstrained);
            eLeft_Center.setDatas(vLeft, eCenter_Left, eCenter_Top, fTopLeft, edge._isReal, edge._isConstrained);
            eCenter_Bot.setDatas(vCenter, eBot_Center, eBot_Right, fBotRight, fBot._isReal);
            eBot_Center.setDatas(vBot, eCenter_Bot, eCenter_Left, fBotLeft, fBot._isReal);
            eCenter_Right.setDatas(vCenter, eRight_Center, eRight_Top, fTopRight, edge._isReal, edge._isConstrained);
            eRight_Center.setDatas(vRight, eCenter_Right, eCenter_Bot, fBotRight, edge._isReal, edge._isConstrained);

            // set the new edge references for the new 4 faces
            fTopLeft.setDatas(eCenter_Top, fTop._isReal);
            fBotLeft.setDatas(eCenter_Left, fBot._isReal);
            fBotRight.setDatas(eCenter_Bot, fBot._isReal);
            fTopRight.setDatas(eCenter_Right, fTop._isReal);

            // check the edge references of LEFT and RIGHT vertices
            if (vLeft._edge == eLeft_Right)
                vLeft.setDatas(eLeft_Center);
            if (vRight._edge == eRight_Left)
                vRight.setDatas(eRight_Center);  // set the new edge and face references for the 4 bounding edges  ;



            eTop_Left._nextLeftEdge = eLeft_Center;
            eTop_Left._leftFace = fTopLeft;
            eLeft_Bot._nextLeftEdge = eBot_Center;
            eLeft_Bot._leftFace = fBotLeft;
            eBot_Right._nextLeftEdge = eRight_Center;
            eBot_Right._leftFace = fBotRight;
            eRight_Top._nextLeftEdge = eTop_Center;
            eRight_Top._leftFace = fTopRight;

            // if the edge was constrained, we must:
            // - add the segments the edge is from to the 2 new
            // - update the segments the edge is from by deleting the old edge and inserting the 2 new
            // - add the segments the edge is from to the new vertex
            if (eLeft_Right._isConstrained)
            {
                var fromSegments = eLeft_Right.fromConstraintSegments;
                eLeft_Center.fromConstraintSegments = fromSegments.Skip(0).Take(fromSegments.Count).ToList();
                eCenter_Left.fromConstraintSegments = eLeft_Center.fromConstraintSegments;
                eCenter_Right.fromConstraintSegments = fromSegments.Skip(0).Take(fromSegments.Count).ToList();
                eRight_Center.fromConstraintSegments = eCenter_Right.fromConstraintSegments;

                List<Edge> edges;
                int index;
                for (var i = 0; i < eLeft_Right.fromConstraintSegments.Count; ++i)
                {
                    edges = eLeft_Right.fromConstraintSegments[i]._edges;
                    index = edges.IndexOf(eLeft_Right);
                    if (index != -1)
                    {
                        edges.RemoveAt(index);
                        edges.Insert(index, eLeft_Center);
                        edges.Insert(index + 1, eCenter_Right);
                    }
                    else
                    {
                        index = edges.IndexOf(eRight_Left);
                        edges.RemoveAt(index);
                        edges.Insert(index, eRight_Center);
                        edges.Insert(index + 1, eCenter_Left);
                    }
                }

                vCenter.set_fromConstraintSegments(fromSegments.Skip(0).Take(fromSegments.Count).ToList());
            }  // remove the old LEFT-RIGHT and RIGHT-LEFT edges  



            eLeft_Right.dispose();
            eRight_Left.dispose();
            _edges.RemoveAt(_edges.IndexOf(eLeft_Right));
            _edges.RemoveAt(_edges.IndexOf(eRight_Left));

            // remove the old TOP and BOTTOM faces
            fTop.dispose();
            fBot.dispose();
            _faces.RemoveAt(_faces.IndexOf(fTop));
            _faces.RemoveAt(_faces.IndexOf(fBot));

            // add new bounds references for Delaunay restoring
            __centerVertex = vCenter;
            __edgesToCheck.Add(eTop_Left);
            __edgesToCheck.Add(eLeft_Bot);
            __edgesToCheck.Add(eBot_Right);
            __edgesToCheck.Add(eRight_Top);

            return vCenter;
        }


        public Vertex splitFace(Face face, FixMath.F64 x, FixMath.F64 y)
        {
            // empty old references
            __edgesToCheck.Clear();

            // retrieve useful objects
            var eTop_Left = face._edge;
            var eLeft_Right = eTop_Left._nextLeftEdge;
            var eRight_Top = eLeft_Right._nextLeftEdge;

            var vTop = eTop_Left._originVertex;
            var vLeft = eLeft_Right._originVertex;
            var vRight = eRight_Top._originVertex;

            // create new objects
            var vCenter = new Vertex();

            var eTop_Center = new Edge();
            var eCenter_Top = new Edge();
            var eLeft_Center = new Edge();
            var eCenter_Left = new Edge();
            var eRight_Center = new Edge();
            var eCenter_Right = new Edge();

            var fTopLeft = new Face();
            var fBot = new Face();
            var fTopRight = new Face();

            // add the new vertex
            _vertices.Add(vCenter);

            // add the new edges
            _edges.Add(eTop_Center);
            _edges.Add(eCenter_Top);
            _edges.Add(eLeft_Center);
            _edges.Add(eCenter_Left);
            _edges.Add(eRight_Center);
            _edges.Add(eCenter_Right);

            // add the new faces
            _faces.Add(fTopLeft);
            _faces.Add(fBot);
            _faces.Add(fTopRight);

            // set pos and edge reference for the new CENTER vertex
            vCenter.setDatas(eCenter_Top);
            vCenter._pos.X = x;
            vCenter._pos.Y = y;

            // set the new vertex, edge and face references for the new 6 center crossing edges
            eTop_Center.setDatas(vTop, eCenter_Top, eCenter_Right, fTopRight);
            eCenter_Top.setDatas(vCenter, eTop_Center, eTop_Left, fTopLeft);
            eLeft_Center.setDatas(vLeft, eCenter_Left, eCenter_Top, fTopLeft);
            eCenter_Left.setDatas(vCenter, eLeft_Center, eLeft_Right, fBot);
            eRight_Center.setDatas(vRight, eCenter_Right, eCenter_Left, fBot);
            eCenter_Right.setDatas(vCenter, eRight_Center, eRight_Top, fTopRight);

            // set the new edge references for the new 3 faces
            fTopLeft.setDatas(eCenter_Top);
            fBot.setDatas(eCenter_Left);
            fTopRight.setDatas(eCenter_Right);

            // set the new edge and face references for the 3 bounding edges
            eTop_Left._nextLeftEdge = eLeft_Center;
            eTop_Left._leftFace = fTopLeft;
            eLeft_Right._nextLeftEdge = eRight_Center;
            eLeft_Right._leftFace = fBot;
            eRight_Top._nextLeftEdge = eTop_Center;
            eRight_Top._leftFace = fTopRight;

            // we remove the old face
            face.dispose();
            _faces.RemoveAt(_faces.IndexOf(face));

            // add new bounds references for Delaunay restoring
            __centerVertex = vCenter;
            __edgesToCheck.Add(eTop_Left);
            __edgesToCheck.Add(eLeft_Right);
            __edgesToCheck.Add(eRight_Top);

            return vCenter;
        }


        public void restoreAsDelaunay()
        {
            Edge edge = null;
            while (__edgesToCheck.Count > 0)
            {
                edge = __edgesToCheck[0];
                __edgesToCheck.RemoveAt(0);
                if (edge._isReal && !edge._isConstrained && !Geom2D.isDelaunay(edge))
                {
                    if (edge._nextLeftEdge.get_destinationVertex() == __centerVertex)
                    {
                        __edgesToCheck.Add(edge.get_nextRightEdge());
                        __edgesToCheck.Add(edge.get_prevRightEdge());
                    }
                    else
                    {
                        __edgesToCheck.Add(edge.get_nextLeftEdge());
                        __edgesToCheck.Add(edge.get_prevLeftEdge());
                    }
                    flipEdge(edge);
                }
            }
        }

        // Delete a vertex IF POSSIBLE and then fill the hole with a new triangulation.
        // A vertex can be deleted if:
        // - it is free of constraint segment (no adjacency to any constrained edge)
        // - it is adjacent to exactly 2 contrained edges and is not an end point of any constraint segment
        public bool deleteVertex(Vertex vertex)
        {
            //Debug.trace("tryToDeleteVertex id " + vertex.id);
            int i;
            bool freeOfConstraint;
            var iterEdges = new FromVertexToOutgoingEdges();
            iterEdges.set_fromVertex(vertex);
            iterEdges.realEdgesOnly = false;
            Edge edge;
            var outgoingEdges = new List<Edge>();

            freeOfConstraint = vertex._fromConstraintSegments.Count == 0;

            //Debug.trace("  -> freeOfConstraint " + freeOfConstraint);

            var bound = new List<Edge>();

            // declares moved out of if loop so haxe compiler knows they exist?
            var realA = false;
            var realB = false;
            var boundA = new List<Edge>();
            var boundB = new List<Edge>();

            if (freeOfConstraint)
            {
                while ((edge = iterEdges.next()) != null)
                {
                    outgoingEdges.Add(edge);
                    bound.Add(edge.get_nextLeftEdge());
                }
            }
            else
            {
                // we check if the vertex is an end point of a constraint segment
                List<Edge> edges;
                for (i = 0; i < vertex._fromConstraintSegments.Count; ++i)
                {
                    edges = vertex._fromConstraintSegments[i].get_edges();
                    if (edges[0].get_originVertex() == vertex || edges[edges.Count - 1].get_destinationVertex() == vertex)
                    {
                        //Debug.trace("  -> is end point of a constraint segment");
                        return false;
                    }
                }  // we check the count of adjacent constrained edges  



                var count = 0;
                while ((edge = iterEdges.next()) != null)
                {
                    outgoingEdges.Add(edge);

                    if (edge._isConstrained)
                    {
                        count++;
                        if (count > 2)
                        {
                            //Debug.trace("  -> count of adjacent constrained edges " + count);
                            return false;
                        }
                    }
                }  //Debug.trace("process vertex deletion");    // if not disqualified, then we can process  




                /// TODO: Moved out of if loop so can be referenced later, not sure of full consequence
                boundA = new List<Edge>();
                boundB = new List<Edge>();
                Edge constrainedEdgeA = null;
                Edge constrainedEdgeB = null;
                var edgeA = new Edge();
                var edgeB = new Edge();
                /// TODO: Moved out of if loop so can be referenced later, not sure of full consequence
                ///var realA : Bool;
                ///var realB : Bool;
                _edges.Add(edgeA);
                _edges.Add(edgeB);
                for (i = 0; i < outgoingEdges.Count; ++i)
                {
                    edge = outgoingEdges[i];
                    if (edge._isConstrained)
                    {
                        if (constrainedEdgeA == null)
                        {
                            edgeB.setDatas(edge.get_destinationVertex(), edgeA, null, null, true, true);
                            boundA.Add(edgeA);
                            boundA.Add(edge.get_nextLeftEdge());
                            boundB.Add(edgeB);
                            constrainedEdgeA = edge;
                        }
                        else if (constrainedEdgeB == null)
                        {
                            edgeA.setDatas(edge.get_destinationVertex(), edgeB, null, null, true, true);
                            boundB.Add(edge.get_nextLeftEdge());
                            constrainedEdgeB = edge;
                        }
                    }
                    else
                    {
                        if (constrainedEdgeA == null)
                            boundB.Add(edge.get_nextLeftEdge());
                        else if (constrainedEdgeB == null)
                            boundA.Add(edge.get_nextLeftEdge());
                        else
                            boundB.Add(edge.get_nextLeftEdge());
                    }
                }  // keep infos about reality  



                realA = constrainedEdgeA.get_leftFace()._isReal;
                realB = constrainedEdgeB.get_leftFace()._isReal;

                // we update the segments infos
                edgeA.fromConstraintSegments = constrainedEdgeA.fromConstraintSegments.Skip(0).Take(constrainedEdgeA.fromConstraintSegments.Count).ToList();
                edgeB.fromConstraintSegments = edgeA.fromConstraintSegments;
                int index;
                for (i = 0; i < vertex._fromConstraintSegments.Count; ++i)
                {
                    edges = vertex._fromConstraintSegments[i]._edges;
                    index = edges.IndexOf(constrainedEdgeA);
                    if (index != -1)
                    {
                        edges.RemoveRange(index - 1, 2);
                        //TODO: check logic of insert
                        edges.Insert(index - 1, edgeA);
                    }
                    else
                    {
                        var index2 = edges.IndexOf(constrainedEdgeB) - 1;
                        edges.RemoveRange(index2, 2);
                        edges.Insert(index2, edgeB);
                    }
                }
            }  // Deletion of old faces and edges  



            Face faceToDelete;
            for (i = 0; i < outgoingEdges.Count; ++i)
            {
                edge = outgoingEdges[i];

                faceToDelete = edge.get_leftFace();
                _faces.RemoveAt(_faces.IndexOf(faceToDelete));
                faceToDelete.dispose();

                edge.get_destinationVertex().set_edge(edge.get_nextLeftEdge());

                _edges.RemoveAt(_edges.IndexOf(edge._oppositeEdge));
                edge._oppositeEdge.dispose();
                _edges.RemoveAt(_edges.IndexOf(edge));
                edge.dispose();
            }

            _vertices.RemoveAt(_vertices.IndexOf(vertex));
            vertex.dispose();

            // finally we triangulate
            if (freeOfConstraint)
            {
                //Debug.trace("trigger single hole triangulation");
                triangulate(bound, true);
            }
            else
            {
                //Debug.trace("trigger dual holes triangulation");
                triangulate(boundA, realA);
                triangulate(boundB, realB);
            }  //check();  



            return true;
        }


        // untriangulate is usually used while a new edge insertion in order to delete the intersected edges
        // edgesList is a list of chained edges oriented from right to left
        void untriangulate(List<Edge> edgesList)
        {
            // we clean useless faces and adjacent vertices
            var i = 0;
            var verticesCleaned = new HashSet<Vertex>();
            Edge currEdge;
            Edge outEdge;
            for (i = 0; i < edgesList.Count; ++i)
            {
                currEdge = edgesList[i];
                //
                if (!verticesCleaned.Contains(currEdge.get_originVertex()))
                {
                    currEdge.get_originVertex().set_edge(currEdge.get_prevLeftEdge()._oppositeEdge);
                    verticesCleaned.Add(currEdge._originVertex);
                }
                if (!verticesCleaned.Contains(currEdge.get_destinationVertex()))
                {
                    currEdge.get_destinationVertex().set_edge(currEdge.get_nextLeftEdge());
                    verticesCleaned.Add(currEdge.get_destinationVertex());
                }  //  

                _faces.RemoveAt(_faces.IndexOf(currEdge.get_leftFace()));
                currEdge.get_leftFace().dispose();
                if (i == edgesList.Count - 1)
                {
                    _faces.RemoveAt(_faces.IndexOf(currEdge.get_rightFace()));
                    currEdge.get_rightFace().dispose();
                }  //  
            }  // finally we delete the intersected edges  



            for (i = 0; i < edgesList.Count; ++i)
            {
                currEdge = edgesList[i];
                _edges.RemoveAt(_edges.IndexOf(currEdge._oppositeEdge));
                _edges.RemoveAt(_edges.IndexOf(currEdge));
                currEdge._oppositeEdge.dispose();
                currEdge.dispose();
            }
        }


        // triangulate is usually used to fill the hole after deletion of a vertex from mesh or after untriangulation
        // - bounds is the list of edges in CCW bounding the surface to retriangulate,
        void triangulate(List<Edge> bound, bool isReal)
        {
            if (bound.Count < 2)
            {
                Debug.LogError("BREAK ! the hole has less than 2 edges");
                return;
            }
            // if the hole is a 2 edges polygon, we have a big problem
            else if (bound.Count == 2)
            {
                //throw new Error("BREAK ! the hole has only 2 edges! " + "  - edge0: " + bound[0].originVertex.id + " -> " + bound[0].destinationVertex.id + "  - edge1: " +  bound[1].originVertex.id + " -> " + bound[1].destinationVertex.id);
                Debug.LogError("BREAK ! the hole has only 2 edges");
                Debug.LogError("  - edge0: " + bound[0].get_originVertex().get_id() + " -> " + bound[0].get_destinationVertex().get_id());
                Debug.LogError("  - edge1: " + bound[1].get_originVertex().get_id() + " -> " + bound[1].get_destinationVertex().get_id());
                return;
            }
            // if the hole is a 3 edges polygon:
            else if (bound.Count == 3)
            {
                /*Debug.trace("the hole is a 3 edges polygon");
                Debug.trace("  - edge0: " + bound[0].originVertex.id + " -> " + bound[0].destinationVertex.id);
                Debug.trace("  - edge1: " + bound[1].originVertex.id + " -> " + bound[1].destinationVertex.id);
                Debug.trace("  - edge2: " + bound[2].originVertex.id + " -> " + bound[2].destinationVertex.id);*/
                var f = new Face();
                f.setDatas(bound[0], isReal);
                _faces.Add(f);
                bound[0].set_leftFace(f);
                bound[1].set_leftFace(f);
                bound[2].set_leftFace(f);
                bound[0].set_nextLeftEdge(bound[1]);
                bound[1].set_nextLeftEdge(bound[2]);
                bound[2].set_nextLeftEdge(bound[0]);
            }
            // if more than 3 edges, we process recursively:
            else
            {
                //Debug.trace("the hole has " + bound.length + " edges");
                /*for (i in 0...bound.length){
                    //Debug.trace("  - edge " + i + ": " + bound[i].originVertex.id + " -> " + bound[i].destinationVertex.id);

                }*/

                var baseEdge = bound[0];
                var vertexA = baseEdge.get_originVertex();
                var vertexB = baseEdge.get_destinationVertex();
                Vertex vertexC;
                Vertex vertexCheck;
                var circumcenter = FixMath.F64Vec2.Zero;
                var radiusSquared = FixMath.F64.Zero;
                var distanceSquared = FixMath.F64.Zero;
                var isDelaunay = false;
                var index = 0;
                int i;
                for (i = 2; i < bound.Count; ++i)
                {
                    vertexC = bound[i].get_originVertex();
                    if (Geom2D.getRelativePosition2(vertexC._pos.X, vertexC._pos.Y, baseEdge) == 1)
                    {
                        index = i;
                        isDelaunay = true;
                        Geom2D.getCircumcenter(vertexA._pos.X, vertexA._pos.Y, vertexB._pos.X, vertexB._pos.Y, vertexC._pos.X, vertexC._pos.Y, out circumcenter);
                        radiusSquared = (vertexA._pos.X - circumcenter.X) * (vertexA._pos.X - circumcenter.X) + (vertexA._pos.Y - circumcenter.Y) * (vertexA._pos.Y - circumcenter.Y);
                        // for perfect regular n-sides polygons, checking strict delaunay circumcircle condition is not possible, so we substract EPSILON to circumcircle radius:
                        radiusSquared -= Constants.EPSILON_SQUARED;
                        for (var j = 2; j < bound.Count; ++j)
                        {
                            if (j != i)
                            {
                                vertexCheck = bound[j].get_originVertex();
                                distanceSquared = (vertexCheck._pos.X - circumcenter.X) * (vertexCheck._pos.X - circumcenter.X) + (vertexCheck._pos.Y - circumcenter.Y) * (vertexCheck._pos.Y - circumcenter.Y);
                                if (distanceSquared < radiusSquared)
                                {
                                    isDelaunay = false;
                                    break;
                                }
                            }
                        }

                        if (isDelaunay)
                            break;
                    }
                }

                if (!isDelaunay)
                {
                    // for perfect regular n-sides polygons, checking delaunay circumcircle condition is not possible
                    Debug.LogError("NO DELAUNAY FOUND");
                    var s = "";
                    for (i = 0; i < bound.Count; ++i)
                    {
                        s += bound[i].get_originVertex()._pos.X + " , ";
                        s += bound[i].get_originVertex()._pos.Y + " , ";
                        s += bound[i].get_destinationVertex()._pos.X + " , ";
                        s += bound[i].get_destinationVertex()._pos.Y + " , ";
                    }  //Debug.trace(s);  


                    index = 2;
                }  //Debug.trace("index " + index + " on " + bound.length);  


                Edge edgeA = null;
                Edge edgeAopp = null;
                Edge edgeB = null;
                Edge edgeBopp;
                List<Edge> boundA;
                List<Edge> boundM;

                //TODO: is this correct??? should it be at **
                List<Edge> boundB;

                if (index < (bound.Count - 1))
                {
                    edgeA = new Edge();
                    edgeAopp = new Edge();
                    _edges.Add(edgeA);
                    _edges.Add(edgeAopp);
                    edgeA.setDatas(vertexA, edgeAopp, null, null, isReal, false);
                    edgeAopp.setDatas(bound[index].get_originVertex(), edgeA, null, null, isReal, false);
                    boundA = bound.Skip(index).Take(bound.Count - index).ToList();
                    boundA.Add(edgeA);
                    triangulate(boundA, isReal);
                }

                if (index > 2)
                {
                    edgeB = new Edge();
                    edgeBopp = new Edge();
                    _edges.Add(edgeB);
                    _edges.Add(edgeBopp);
                    edgeB.setDatas(bound[1].get_originVertex(), edgeBopp, null, null, isReal, false);
                    edgeBopp.setDatas(bound[index].get_originVertex(), edgeB, null, null, isReal, false);
                    boundB = bound.Skip(1).Take(index - 1).ToList();
                    boundB.Add(edgeBopp);
                    triangulate(boundB, isReal);
                }
                // **
                if (index == 2)
                {
                    boundM = new List<Edge> { baseEdge, bound[1], edgeAopp };
                }
                else if (index == (bound.Count - 1))
                {
                    boundM = new List<Edge> { baseEdge, edgeB, bound[index] };
                }
                else
                {
                    boundM = new List<Edge> { baseEdge, edgeB, edgeAopp };
                }

                triangulate(boundM, isReal);
            }
        }


        public int findPositionFromBounds(FixMath.F64 x, FixMath.F64 y)
        {
            if ((x <= 0))
            {
                if ((y <= 0))
                {
                    return 1;
                }
                else if ((y >= this._height))
                {
                    return 7;
                }
                else
                {
                    return 8;
                }

            }
            else if ((x >= this._width))
            {
                if ((y <= 0))
                {
                    return 3;
                }
                else if ((y >= this._height))
                {
                    return 5;
                }
                else
                {
                    return 4;
                }

            }
            else if ((y <= 0))
            {
                return 2;
            }
            else if ((y >= this._height))
            {
                return 6;
            }
            else
            {
                return 0;
            }
        }


        public void debug()
        {
            int i;
            for (i = 0; i < _vertices.Count; ++i)
            {
                Debug.Log("-- vertex " + _vertices[i]._id);
                Debug.Log("  edge " + _vertices[i]._edge._id + " - " + _vertices[i]._edge);
                Debug.Log("  edge isReal: " + _vertices[i]._edge._isReal);
            }
            for (i = 0; i < _edges.Count; ++i)
            {
                Debug.Log("-- edge " + _edges[i]);
                Debug.Log("  isReal " + _edges[i]._id + " - " + _edges[i]._isReal);
                Debug.Log("  nextLeftEdge " + _edges[i].get_nextLeftEdge());
                Debug.Log("  oppositeEdge " + _edges[i].get_oppositeEdge());
            }

        }


        public (List<Vertex>, List<Edge>) getVerticesAndEdges()
        {
            var vertices = new List<Vertex>();
            var edges = new List<Edge>();

            Vertex vertex;
            Edge incomingEdge;

            var iterVertices = new FromMeshToVertices();
            iterVertices.set_fromMesh(this);

            var iterEdges = new FromVertexToIncomingEdges();
            var dictVerticesDone = new HashSet<Vertex>();

            while ((vertex = iterVertices.next()) != null)
            {
                dictVerticesDone.Add(vertex);
                if (!vertexIsInsideAABB(vertex, this))
                    continue;

                vertices.Add(vertex);

                iterEdges.set_fromVertex(vertex);
                while ((incomingEdge = iterEdges.next()) != null)
                {
                    if (!dictVerticesDone.Contains(incomingEdge.get_originVertex()))
                    {
                        edges.Add(incomingEdge);
                    }
                }
            }
            return (vertices, edges);
        }


        public bool vertexIsInsideAABB(Vertex vertex, Mesh mesh)
        {
            return !(vertex._pos.X < 0 || vertex._pos.X > mesh._width || vertex._pos.Y < 0 || vertex._pos.Y > mesh._height);
        }
    }
}
