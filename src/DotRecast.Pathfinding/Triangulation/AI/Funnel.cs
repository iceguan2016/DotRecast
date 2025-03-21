
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Reflection;
using Game.Utils;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Math;

namespace Pathfinding.Triangulation.AI
{
    public class Funnel
    {
        public class Point2D
        {
            public Point2D(FixMath.F64? x = null, FixMath.F64? y = null)
            {
                _pos = new FixMath.F64Vec2(x ?? FixMath.F64.Zero, y ?? FixMath.F64.Zero);
            }
            public Point2D(FixMath.F64Vec2 p)
            {
                _pos = p;
            }


            public FixMath.F64 x
            {
                get
                {
                    return _pos.X;
                }

                set
                {
                    _pos.X = value;
                }
            }

            public FixMath.F64 y
            {
                get
                {
                    return _pos.Y;
                }

                set
                {
                    _pos.Y = value;
                }
            }

            public FixMath.F64Vec2 _pos;
        }

        public Funnel()
        {
            _poolPoints = new List<Point2D>();
            for (var i = 0; i < _poolPointsSize; ++i)
            {
                _poolPoints.Add(new Point2D());
            }
        }

        private FixMath.F64 _radius = FixMath.F64.Zero;

        private FixMath.F64 _radiusSquared = FixMath.F64.Zero;

        private int _numSamplesCircle = 16;

        private List<FixMath.F64Vec2> _sampleCircle;

        private FixMath.F64 _sampleCircleDistanceSquared = FixMath.F64.Zero;

        public virtual void dispose()
        {
            this._sampleCircle = null;
        }


        public int _poolPointsSize = 3000;

        public List<Point2D> _poolPoints;

        public int _currPoolPointsIndex = 0;

        public Point2D __point;

        public Point2D getPoint(FixMath.F64? x = null, FixMath.F64? y = null)
        {
            __point = _poolPoints[_currPoolPointsIndex];
            __point._pos.X = x ?? FixMath.F64.Zero;
            __point._pos.Y = y ?? FixMath.F64.Zero;

            _currPoolPointsIndex++;
            if (_currPoolPointsIndex == _poolPointsSize)
            {
                _poolPoints.Add(new Point2D());
                _poolPointsSize++;
            }

            return __point;
        }

        public Point2D getCopyPoint(FixMath.F64Vec2 pointToCopy)
        {
            return getPoint(pointToCopy.X, pointToCopy.Y);
        }

        public FixMath.F64 get_radius()
        {
            return this._radius;
        }

        public FixMath.F64 set_radius(FixMath.F64 value)
        {
            _radius = FixMath.F64.Max(FixMath.F64.Zero, value);
            _radiusSquared = _radius * _radius;
            _sampleCircle = new List<FixMath.F64Vec2>();
            if (_radius == 0)
                return FixMath.F64.Zero;
            for (var i = 0; i < _numSamplesCircle; ++i)
            {
                _sampleCircle.Add(new FixMath.F64Vec2(
                    _radius * FixMath.F64.Cos(-2 * FixMath.F64.Pi * i / _numSamplesCircle),
                    _radius * FixMath.F64.Sin(-2 * FixMath.F64.Pi * i / _numSamplesCircle)));
            }
            _sampleCircleDistanceSquared = (_sampleCircle[0].X - _sampleCircle[1].X) * (_sampleCircle[0].X - _sampleCircle[1].X) + (_sampleCircle[0].Y - _sampleCircle[1].Y) * (_sampleCircle[0].Y - _sampleCircle[1].Y);
            return _radius;
        }

        static FixMath.F64 RADIUS_SCALE = FixMath.F64.FromDouble(1.01);
        public void findPath(FixMath.F64 fromX, FixMath.F64 fromY
                            , FixMath.F64 toX, FixMath.F64 toY
                            , List<Face> listFaces
                            , List<Edge> listEdges
                            , List<FixMath.F64> resultPath)
        {
            _currPoolPointsIndex = 0;

            // we check the start and goal
            if (_radius > 0)
            {
                var checkFace = listFaces[0];
                FixMath.F64 distanceSquared;
                FixMath.F64 inv_distance;
                FixMath.F64Vec2 p1;
                FixMath.F64Vec2 p2;
                FixMath.F64Vec2 p3;
                p1 = checkFace._edge.get_originVertex()._pos;
                p2 = checkFace._edge.get_destinationVertex()._pos;
                p3 = checkFace._edge.get_nextLeftEdge().get_destinationVertex()._pos;
                distanceSquared = (p1.X - fromX) * (p1.X - fromX) + (p1.Y - fromY) * (p1.Y - fromY);
                if (distanceSquared <= _radiusSquared)
                {
                    inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                    fromX = _radius * RADIUS_SCALE * ((fromX - p1.X) * inv_distance) + p1.X;
                    fromY = _radius * RADIUS_SCALE * ((fromY - p1.Y) * inv_distance) + p1.Y;
                }
                else
                {
                    distanceSquared = (p2.X - fromX) * (p2.X - fromX) + (p2.Y - fromY) * (p2.Y - fromY);
                    if (distanceSquared <= _radiusSquared)
                    {
                        inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                        fromX = _radius * RADIUS_SCALE * ((fromX - p2.X) * inv_distance) + p2.X;
                        fromY = _radius * RADIUS_SCALE * ((fromY - p2.Y) * inv_distance) + p2.Y;
                    }
                    else
                    {
                        distanceSquared = (p3.X - fromX) * (p3.X - fromX) + (p3.Y - fromY) * (p3.Y - fromY);
                        if (distanceSquared <= _radiusSquared)
                        {
                            inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                            fromX = _radius * RADIUS_SCALE * ((fromX - p3.X) * inv_distance) + p3.X;
                            fromY = _radius * RADIUS_SCALE * ((fromY - p3.Y) * inv_distance) + p3.Y;
                        }
                    }
                }  //  

                checkFace = listFaces[listFaces.Count - 1];
                p1 = checkFace._edge.get_originVertex()._pos;
                p2 = checkFace._edge.get_destinationVertex()._pos;
                p3 = checkFace._edge.get_nextLeftEdge().get_destinationVertex()._pos;
                distanceSquared = (p1.X - toX) * (p1.X - toX) + (p1.Y - toY) * (p1.Y - toY);
                if (distanceSquared <= _radiusSquared)
                {
                    inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                    toX = _radius * RADIUS_SCALE * ((toX - p1.X) * inv_distance) + p1.X;
                    toY = _radius * RADIUS_SCALE * ((toY - p1.Y) * inv_distance) + p1.Y;
                }
                else
                {
                    distanceSquared = (p2.X - toX) * (p2.X - toX) + (p2.Y - toY) * (p2.Y - toY);
                    if (distanceSquared <= _radiusSquared)
                    {
                        inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                        toX = _radius * RADIUS_SCALE * ((toX - p2.X) * inv_distance) + p2.X;
                        toY = _radius * RADIUS_SCALE * ((toY - p2.Y) * inv_distance) + p2.Y;
                    }
                    else
                    {
                        distanceSquared = (p3.X - toX) * (p3.X - toX) + (p3.Y - toY) * (p3.Y - toY);
                        if (distanceSquared <= _radiusSquared)
                        {
                            inv_distance = FixMath.F64.RSqrtFast(distanceSquared);
                            toX = _radius * RADIUS_SCALE * ((toX - p3.X) * inv_distance) + p3.X;
                            toY = _radius * RADIUS_SCALE * ((toY - p3.Y) * inv_distance) + p3.Y;
                        }
                    }
                }
            }  // we build starting and ending points  

            var startPoint = getPoint(fromX, fromY);
            var endPoint = getPoint(toX, toY);

            if (listFaces.Count == 1)
            {
                resultPath.Add(startPoint.x);
                resultPath.Add(startPoint.y);
                resultPath.Add(endPoint.x);
                resultPath.Add(endPoint.y);
                return;
            }  // useful  

            int i;
            int j;
            int k;
            Edge currEdge = null;
            Vertex currVertex = null;
            int direction;

            // first we skip the first face and first edge if the starting point lies on the first interior edge:

            // switch on intersection 
            //TODO: check if this needs ENull (/not EEdge) checks
            var intersection = Geom2D.isInFace(fromX, fromY, listFaces[0]);
            if (intersection is Intersection_EEdge e)
            {
                if (listEdges[0] == e.edge)
                {
                    listEdges.RemoveAt(0);
                    listFaces.RemoveAt(0);
                }
            }
            // our funnels, inited with starting point  



            var funnelLeft = new List<Point2D>();
            var funnelRight = new List<Point2D>();
            funnelLeft.Add(startPoint);
            funnelRight.Add(startPoint);

            // useful to keep track of done vertices and compare the sides
            var verticesDoneSide = new Dictionary<Vertex, int>();

            // we extract the vertices positions and sides from the edges list
            var pointsList = new List<Point2D>();
            var pointSides = new Dictionary<Point2D, int>();
            // we keep the successor relation in a dictionnary
            var pointSuccessor = new Dictionary<Point2D, Point2D>();
            //
            pointSides[startPoint] = 0;
            // we begin with the vertices in first edge
            currEdge = listEdges[0];
            var relativPos = Geom2D.getRelativePosition2(fromX, fromY, currEdge);
            Point2D prevPoint;
            Point2D newPointA;
            Point2D newPointB;
            newPointA = getCopyPoint(currEdge.get_destinationVertex()._pos);
            newPointB = getCopyPoint(currEdge.get_originVertex()._pos);

            pointsList.Add(newPointA);
            pointsList.Add(newPointB);
            pointSuccessor[startPoint] = newPointA;
            pointSuccessor[newPointA] = newPointB;
            prevPoint = newPointB;
            if (relativPos == 1)
            {
                pointSides[newPointA] = 1;
                pointSides[newPointB] = -1;
                verticesDoneSide[currEdge.get_destinationVertex()] = 1;
                verticesDoneSide[currEdge.get_originVertex()] = -1;
            }
            else if (relativPos == -1)
            {// then we iterate through the edges
                pointSides[newPointA] = -1;
                pointSides[newPointB] = 1;
                verticesDoneSide[currEdge.get_destinationVertex()] = -1;
                verticesDoneSide[currEdge.get_originVertex()] = 1;
            }



            var fromVertex = listEdges[0].get_originVertex();
            var fromFromVertex = listEdges[0].get_destinationVertex();
            for (i = 1; i < listEdges.Count; ++i)
            {
                // we identify the current vertex and his origin vertex
                currEdge = listEdges[i];
                if (currEdge.get_originVertex() == fromVertex)
                {
                    currVertex = currEdge.get_destinationVertex();
                }
                else if (currEdge.get_destinationVertex() == fromVertex)
                {
                    currVertex = currEdge.get_originVertex();
                }
                else if (currEdge.get_originVertex() == fromFromVertex)
                {
                    currVertex = currEdge.get_destinationVertex();
                    fromVertex = fromFromVertex;
                }
                else if (currEdge.get_destinationVertex() == fromFromVertex)
                {
                    currVertex = currEdge.get_originVertex();
                    fromVertex = fromFromVertex;
                }
                else
                {
                    Debug.LogError("IMPOSSIBLE TO IDENTIFY THE VERTEX !!!");
                }

                newPointA = getCopyPoint(currVertex._pos);
                pointsList.Add(newPointA);
                direction = -verticesDoneSide[fromVertex];
                pointSides[newPointA] = direction;
                pointSuccessor[prevPoint] = newPointA;
                verticesDoneSide[currVertex] = direction;
                prevPoint = newPointA;
                fromFromVertex = fromVertex;
                fromVertex = currVertex;
            }  // we then we add the end point  

            pointSuccessor[prevPoint] = endPoint;
            pointSides[endPoint] = 0;

            /*
            debugSurface.graphics.clear();
            debugSurface.graphics.lineStyle(1, 0x0000FF);
            var ppp1:Point = startPoint;
            var ppp2:Point = pointSuccessor[ppp1];
            while (ppp2)
            {
            debugSurface.graphics.moveTo(ppp1.x, ppp1.y+2);
            debugSurface.graphics.lineTo(ppp2.x, ppp2.y+2);
            debugSurface.graphics.drawCircle(ppp2.x, ppp2.y, 3);
            ppp1 = ppp2;
            ppp2 = pointSuccessor[ppp2];
            }

            debugSurface.graphics.lineStyle(1, 0x00FF00);
            for (i=1 ; i<pointsList.length ; i++)
            {
            debugSurface.graphics.moveTo(pointsList[i-1].x+2, pointsList[i-1].y);
            debugSurface.graphics.lineTo(pointsList[i].x+2, pointsList[i].y);
            }
            */

            // we will keep the points and funnel sides of the optimized path
            var pathPoints = new List<Point2D>();
            var pathSides = new Dictionary<Point2D, int>();
            pathPoints.Add(startPoint);
            pathSides[startPoint] = 0;

            // now we process the points by order
            Point2D currPos = null;
            for (i = 0; i < pointsList.Count; ++i)
            {
                currPos = pointsList[i];

                // we identify the current vertex funnel's position by the position of his origin vertex
                if (pointSides[currPos] == -1)
                {
                    // current vertex is at right
                    //Debug.trace("current vertex is at right");
                    j = funnelLeft.Count - 2;
                    while (j >= 0)
                    {
                        direction = Geom2D.getDirection(funnelLeft[j].x, funnelLeft[j].y, funnelLeft[j + 1].x, funnelLeft[j + 1].y, currPos.x, currPos.y);
                        if (direction != -1)
                        {
                            //Debug.trace("funnels are crossing");

                            funnelLeft.RemoveAt(0);
                            for (k = 0; k < j; ++k)
                            {
                                pathPoints.Add(funnelLeft[0]);
                                pathSides[funnelLeft[0]] = 1;
                                funnelLeft.RemoveAt(0);
                            }
                            pathPoints.Add(funnelLeft[0]);
                            pathSides[funnelLeft[0]] = 1;
                            funnelRight.Clear();
                            funnelRight.Add(funnelLeft[0]);
                            funnelRight.Add(currPos);
                            break;
                        }
                        j--;
                    }

                    funnelRight.Add(currPos);
                    j = funnelRight.Count - 3;
                    while (j >= 0)
                    {
                        direction = Geom2D.getDirection(funnelRight[j].x, funnelRight[j].y, funnelRight[j + 1].x, funnelRight[j + 1].y, currPos.x, currPos.y);
                        if (direction == -1)
                            break;
                        else
                        {
                            funnelRight.RemoveAt(j + 1);
                        }
                        j--;
                    }
                }
                else
                {
                    // current vertex is at left
                    j = funnelRight.Count - 2;
                    while (j >= 0)
                    {
                        direction = Geom2D.getDirection(funnelRight[j].x, funnelRight[j].y, funnelRight[j + 1].x, funnelRight[j + 1].y, currPos.x, currPos.y);
                        if (direction != 1)
                        {
                            funnelRight.RemoveAt(0);
                            for (k = 0; k < j; ++k)
                            {
                                pathPoints.Add(funnelRight[0]);
                                pathSides[funnelRight[0]] = -1;
                                funnelRight.RemoveAt(0);
                            }
                            pathPoints.Add(funnelRight[0]);
                            pathSides[funnelRight[0]] = -1;
                            funnelLeft.Clear();
                            funnelLeft.Add(funnelRight[0]);
                            funnelLeft.Add(currPos);
                            break;
                        }
                        j--;
                    }

                    funnelLeft.Add(currPos);
                    j = funnelLeft.Count - 3;
                    while (j >= 0)
                    {
                        direction = Geom2D.getDirection(funnelLeft[j].x, funnelLeft[j].y, funnelLeft[j + 1].x, funnelLeft[j + 1].y, currPos.x, currPos.y);
                        if (direction == 1)
                            break;
                        else
                        {
                            funnelLeft.RemoveAt(j + 1);
                        }
                        j--;
                    }
                }
            }  // check if the goal is blocked by one funnel's right vertex  



            var blocked = false;
            //Debug.trace("check if the goal is blocked by one funnel right vertex");
            j = funnelRight.Count - 2;
            while (j >= 0)
            {
                direction = Geom2D.getDirection(funnelRight[j].x, funnelRight[j].y, funnelRight[j + 1].x, funnelRight[j + 1].y, toX, toY);
                //Debug.trace("dir" + funnelRight[j].x + "," + funnelRight[j].y + " " + funnelRight[j+1].x + "," + funnelRight[j+1].y + " " + toX + "," + toY);
                if (direction != 1)
                {
                    //Debug.trace("goal access right blocked");
                    // access blocked
                    funnelRight.RemoveAt(0);
                    for (k = 0; k < j + 1; ++k)
                    {
                        pathPoints.Add(funnelRight[0]);
                        pathSides[funnelRight[0]] = -1;
                        funnelRight.RemoveAt(0);
                    }
                    pathPoints.Add(endPoint);
                    pathSides[endPoint] = 0;
                    blocked = true;
                    break;
                }
                j--;
            }

            if (!blocked)
            {
                // check if the goal is blocked by one funnel's left vertex
                //Debug.trace("check if the goal is blocked by one funnel left vertex");
                j = funnelLeft.Count - 2;
                while (j >= 0)
                {
                    direction = Geom2D.getDirection(funnelLeft[j].x, funnelLeft[j].y, funnelLeft[j + 1].x, funnelLeft[j + 1].y, toX, toY);
                    //Debug.trace("dir " + funnelLeft[j].x + "," + funnelLeft[j].y + " " + funnelLeft[j+1].x + "," + funnelLeft[j+1].y + " " + toX + "," + toY);
                    if (direction != -1)
                    {
                        //Debug.trace("goal access left blocked");
                        // access blocked
                        funnelLeft.RemoveAt(0);
                        for (k = 0; k < j + 1; ++k)
                        {
                            pathPoints.Add(funnelLeft[0]);
                            pathSides[funnelLeft[0]] = 1;
                            funnelLeft.RemoveAt(0);
                        }

                        pathPoints.Add(endPoint);
                        pathSides[endPoint] = 0;
                        blocked = true;
                        break;
                    }
                    j--;
                }
            }  // if not blocked, we consider the direct path  



            if (!blocked)
            {
                pathPoints.Add(endPoint);
                pathSides[endPoint] = 0;
                blocked = true;
            }  // if radius is non zero  


            var adjustedPoints = new List<Point2D>();
            if (_radius > 0)
            {

                var newPath = new List<Point2D>();

                if (pathPoints.Count == 2)
                {
                    adjustWithTangents(pathPoints[0], false, pathPoints[1], false, pointSides, pointSuccessor, newPath, adjustedPoints);
                }
                else if (pathPoints.Count > 2)
                {
                    // tangent from start point to 2nd point
                    adjustWithTangents(pathPoints[0], false, pathPoints[1], true, pointSides, pointSuccessor, newPath, adjustedPoints);

                    // tangents for intermediate points
                    if (pathPoints.Count > 3)
                    {
                        for (i = 1; i < pathPoints.Count - 3 + 1; ++i)
                        {
                            adjustWithTangents(pathPoints[i], true, pathPoints[i + 1], true, pointSides, pointSuccessor, newPath, adjustedPoints);
                        }
                    }  // tangent from last-1 point to end point  



                    var pathLength = pathPoints.Count;
                    adjustWithTangents(pathPoints[pathLength - 2], true, pathPoints[pathLength - 1], false, pointSides, pointSuccessor, newPath, adjustedPoints);
                }

                newPath.Add(endPoint);

                // adjusted path can have useless tangents, we check it
                checkAdjustedPath(newPath, adjustedPoints, pointSides);

                var smoothPoints = new List<Point2D>();
                i = newPath.Count - 2;
                while (i >= 1)
                {
                    smoothAngle(adjustedPoints[i * 2 - 1], newPath[i], adjustedPoints[i * 2], pointSides[newPath[i]], smoothPoints);
                    while (smoothPoints.Count != 0)
                    {
                        var temp = i * 2;
                        // adjustedPoints.splice(temp, 0);
                        adjustedPoints.Insert(temp, smoothPoints[0]);
                        smoothPoints.RemoveAt(0);
                    }
                    i--;
                }
            }
            else
            {
                adjustedPoints = pathPoints;
            }  // extract coordinates  



            for (i = 0; i < adjustedPoints.Count; ++i)
            {
                resultPath.Add(adjustedPoints[i].x);
                resultPath.Add(adjustedPoints[i].y);
            }
        }

        void adjustWithTangents(Point2D p1, bool applyRadiusToP1, Point2D p2, bool applyRadiusToP2, Dictionary<Point2D, int> pointSides, Dictionary<Point2D, Point2D> pointSuccessor, List<Point2D> newPath, List<Point2D> adjustedPoints)
        {
            // we find the tangent T between the points pathPoints[i] - pathPoints[i+1]
            // then we check the unused intermediate points between pathPoints[i] and pathPoints[i+1]
            // if a point P is too close from the segment, we replace T by 2 tangents T1, T2, between the points pathPoints[i] P and P - pathPoints[i+1]

            //Debug.trace("adjustWithTangents");

            var tangentsResult = new List<FixMath.F64>();

            var side1 = pointSides[p1];
            var side2 = pointSides[p2];

            Point2D pTangent1 = null;
            Point2D pTangent2 = null;

            // if no radius application
            if (!applyRadiusToP1 && !applyRadiusToP2)
            {
                //Debug.trace("no radius applied");
                pTangent1 = p1;
                pTangent2 = p2;
            }
            // we apply radius to p2 only
            else if (!applyRadiusToP1)
            {
                //Debug.trace("! applyRadiusToP1");
                if (Geom2D.tangentsPointToCircle(p1.x, p1.y, p2.x, p2.y, _radius, tangentsResult))
                {
                    // p2 lies on the left funnel
                    if (side2 == 1)
                    {
                        pTangent1 = p1;
                        pTangent2 = getPoint(tangentsResult[2], tangentsResult[3]);
                    }
                    // p2 lies on the right funnel
                    else
                    {
                        pTangent1 = p1;
                        pTangent2 = getPoint(tangentsResult[0], tangentsResult[1]);
                    }
                }
                else
                {
                    // Debug.trace("NO TANGENT");
                    return;
                }
            }
            // we apply radius to p1 only
            else if (!applyRadiusToP2)
            {
                //Debug.trace("! applyRadiusToP2");
                if (Geom2D.tangentsPointToCircle(p2.x, p2.y, p1.x, p1.y, _radius, tangentsResult))
                {
                    if (tangentsResult.Count > 0)
                    {
                        // p1 lies on the left funnel
                        if (side1 == 1)
                        {
                            pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                            pTangent2 = p2;
                        }
                        // p1 lies on the right funnel
                        else
                        {
                            pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                            pTangent2 = p2;
                        }
                    }
                }
                else
                {
                    // Debug.trace("NO TANGENT");
                    return;
                }
            }
            // we apply radius to both points
            else
            {
                //Debug.trace("we apply radius to both points");
                // both points lie on left funnel
                if (side1 == 1 && side2 == 1)
                {
                    Geom2D.tangentsParalCircleToCircle(_radius, p1.x, p1.y, p2.x, p2.y, tangentsResult);
                    // we keep the points of the right tangent
                    pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                    pTangent2 = getPoint(tangentsResult[4], tangentsResult[5]);
                }
                // both points lie on right funnel
                else if (side1 == -1 && side2 == -1)
                {
                    Geom2D.tangentsParalCircleToCircle(_radius, p1.x, p1.y, p2.x, p2.y, tangentsResult);
                    // we keep the points of the left tangent
                    pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                    pTangent2 = getPoint(tangentsResult[6], tangentsResult[7]);
                }
                // 1st point lies on left funnel, 2nd on right funnel
                else if (side1 == 1 && side2 == -1)
                {
                    if (Geom2D.tangentsCrossCircleToCircle(_radius, p1.x, p1.y, p2.x, p2.y, tangentsResult))
                    {
                        // we keep the points of the right-left tangent
                        pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                        pTangent2 = getPoint(tangentsResult[6], tangentsResult[7]);
                    }
                    else
                    {
                        // NO TANGENT BECAUSE POINTS TOO CLOSE
                        // A* MUST CHECK THAT !
                        // Debug.trace("NO TANGENT, points are too close for radius");
                        return;
                    }
                }
                // 1st point lies on right funnel, 2nd on left funnel
                else
                {
                    if (Geom2D.tangentsCrossCircleToCircle(_radius, p1.x, p1.y, p2.x, p2.y, tangentsResult))
                    {
                        // we keep the points of the left-right tangent
                        pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                        pTangent2 = getPoint(tangentsResult[4], tangentsResult[5]);
                    }
                    else
                    {
                        // NO TANGENT BECAUSE POINTS TOO CLOSE
                        // A* MUST CHECK THAT !
                        // Debug.trace("NO TANGENT, points are too close for radius");
                        return;
                    }
                }
            }

            var successor = pointSuccessor[p1];
            FixMath.F64 distance;
            while (successor != p2)
            {
                distance = Geom2D.distanceSquaredPointToSegment(successor.x, successor.y, pTangent1.x, pTangent1.y, pTangent2.x, pTangent2.y);
                if (distance < _radiusSquared)
                {
                    adjustWithTangents(p1, applyRadiusToP1, successor, true, pointSides, pointSuccessor, newPath, adjustedPoints);
                    adjustWithTangents(successor, true, p2, applyRadiusToP2, pointSides, pointSuccessor, newPath, adjustedPoints);
                    return;
                }
                else
                {
                    successor = pointSuccessor[successor];
                }
            }
            /*if ( adjustedPoints.length > 0 )
            {
                var distanceSquared:Number;
                var lastPoint:Point = adjustedPoints[adjustedPoints.length-1];
                distanceSquared = (lastPoint.x - pTangent1.x)*(lastPoint.x - pTangent1.x) + (lastPoint.y - pTangent1.y)*(lastPoint.y - pTangent1.y);
                if (distanceSquared <= QEConstants.EPSILON_SQUARED)
                {
                    adjustedPoints.pop();
                    adjustedPoints.push(pTangent2);
                    return;
                }
            }*/    // we check distance in order to remove useless close points due to straight line subdivision  

            adjustedPoints.Add(pTangent1);
            adjustedPoints.Add(pTangent2);
            newPath.Add(p1);
        }

        void checkAdjustedPath(List<Point2D> newPath, List<Point2D> adjustedPoints, Dictionary<Point2D, int> pointSides)
        {

            var needCheck = true;

            Point2D point0;
            int point0Side;
            Point2D point1;
            int point1Side;
            Point2D point2;
            int point2Side;

            Point2D pt1;
            Point2D pt2;
            Point2D pt3;
            FixMath.F64 dot;

            var tangentsResult = new List<FixMath.F64>();
            Point2D pTangent1 = null;
            Point2D pTangent2 = null;

            while (needCheck)
            {
                needCheck = false;
                var i = 2;
                while (i < newPath.Count)
                {
                    point2 = newPath[i];
                    point2Side = pointSides[point2];
                    point1 = newPath[i - 1];
                    point1Side = pointSides[point1];
                    point0 = newPath[i - 2];
                    point0Side = pointSides[point0];

                    if (point1Side == point2Side)
                    {
                        pt1 = adjustedPoints[(i - 2) * 2];
                        pt2 = adjustedPoints[(i - 1) * 2 - 1];
                        pt3 = adjustedPoints[(i - 1) * 2];
                        dot = (pt1.x - pt2.x) * (pt3.x - pt2.x) + (pt1.y - pt2.y) * (pt3.y - pt2.y);
                        if (dot > 0)
                        {
                            //needCheck = true;
                            //Debug.trace("dot > 0");
                            // rework the tangent
                            if (i == 2)
                            {
                                // tangent from start point
                                Geom2D.tangentsPointToCircle(point0.x, point0.y, point2.x, point2.y, _radius, tangentsResult);
                                // p2 lies on the left funnel
                                if (point2Side == 1)
                                {
                                    pTangent1 = point0;
                                    pTangent2 = getPoint(tangentsResult[2], tangentsResult[3]);
                                }
                                else
                                {
                                    pTangent1 = point0;
                                    pTangent2 = getPoint(tangentsResult[0], tangentsResult[1]);
                                }
                            }
                            else if (i == newPath.Count - 1)
                            {
                                // tangent to end point
                                Geom2D.tangentsPointToCircle(point2.x, point2.y, point0.x, point0.y, _radius, tangentsResult);
                                // p1 lies on the left funnel
                                if (point0Side == 1)
                                {
                                    pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                                    pTangent2 = point2;
                                }
                                // p1 lies on the right funnel
                                else
                                {
                                    pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                                    pTangent2 = point2;
                                }
                            }
                            else
                            {
                                // 1st point lies on left funnel, 2nd on right funnel
                                if (point0Side == 1 && point2Side == -1)
                                {
                                    //Debug.trace("point0Side == 1 && point2Side == -1");
                                    Geom2D.tangentsCrossCircleToCircle(_radius, point0.x, point0.y, point2.x, point2.y, tangentsResult);  // we keep the points of the right-left tangent  ;

                                    pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                                    pTangent2 = getPoint(tangentsResult[6], tangentsResult[7]);
                                }
                                // 1st point lies on right funnel, 2nd on left funnel
                                else if (point0Side == -1 && point2Side == 1)
                                {
                                    //Debug.trace("point0Side == -1 && point2Side == 1");
                                    Geom2D.tangentsCrossCircleToCircle(_radius, point0.x, point0.y, point2.x, point2.y, tangentsResult);  // we keep the points of the right-left tangent  ;

                                    pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                                    pTangent2 = getPoint(tangentsResult[4], tangentsResult[5]);
                                }
                                // both points lie on left funnel
                                else if (point0Side == 1 && point2Side == 1)
                                {
                                    //Debug.trace("point0Side == 1 && point2Side == 1");
                                    Geom2D.tangentsParalCircleToCircle(_radius, point0.x, point0.y, point2.x, point2.y, tangentsResult);
                                    // we keep the points of the right tangent
                                    pTangent1 = getPoint(tangentsResult[2], tangentsResult[3]);
                                    pTangent2 = getPoint(tangentsResult[4], tangentsResult[5]);
                                }
                                // both points lie on right funnel
                                else if (point0Side == -1 && point2Side == -1)
                                {
                                    //Debug.trace("point0Side == -1 && point2Side == -1");
                                    Geom2D.tangentsParalCircleToCircle(_radius, point0.x, point0.y, point2.x, point2.y, tangentsResult);
                                    // we keep the points of the right tangent
                                    pTangent1 = getPoint(tangentsResult[0], tangentsResult[1]);
                                    pTangent2 = getPoint(tangentsResult[6], tangentsResult[7]);
                                }
                            }
                            var temp = (i - 2) * 2;
                            adjustedPoints.RemoveAt(temp);
                            adjustedPoints.Insert(temp, pTangent1);
                            temp = i * 2 - 1;
                            adjustedPoints.RemoveAt(temp);
                            adjustedPoints.Insert(temp, pTangent2);

                            // delete useless point
                            newPath.RemoveAt(i - 1);
                            adjustedPoints.RemoveRange((i - 1) * 2 - 1, 2);

                            tangentsResult.Clear();
                            i--;
                        }
                    }
                    i++;
                }
            }
        }

        void smoothAngle(Point2D prevPoint, Point2D pointToSmooth, Point2D nextPoint, int side, List<Point2D> encirclePoints)
        {
            var angleType = Geom2D.getDirection(prevPoint.x, prevPoint.y, pointToSmooth.x, pointToSmooth.y, nextPoint.x, nextPoint.y);

            /*
            Debug.trace("smoothAngle");
            Debug.trace("angleType " + angleType);
            Debug.trace("prevPoint " + prevPoint);
            Debug.trace("pointToSmooth " + pointToSmooth);
            Debug.trace("nextPoint " + nextPoint);
            */

            var distanceSquared = (prevPoint.x - nextPoint.x) * (prevPoint.x - nextPoint.x) + (prevPoint.y - nextPoint.y) * (prevPoint.y - nextPoint.y);
            if (distanceSquared <= _sampleCircleDistanceSquared)
                return;

            var index = 0;
            var side1 = 0;
            var side2 = 0;
            var pointInArea = false;
            var xToCheck = FixMath.F64.Zero;
            var yToCheck = FixMath.F64.Zero;
            for (var i = 0; i < _numSamplesCircle; ++i)
            {
                pointInArea = false;
                xToCheck = pointToSmooth._pos.X + _sampleCircle[i].X;
                yToCheck = pointToSmooth._pos.Y + _sampleCircle[i].Y;
                side1 = Geom2D.getDirection(prevPoint.x, prevPoint.y, pointToSmooth.x, pointToSmooth.y, xToCheck, yToCheck);
                side2 = Geom2D.getDirection(pointToSmooth.x, pointToSmooth.y, nextPoint.x, nextPoint.y, xToCheck, yToCheck);

                // if funnel left
                if (side == 1)
                {
                    //Debug.trace("funnel side is 1");
                    // if angle is < 180
                    if (angleType == -1)
                    {
                        //Debug.trace("angle type is -1");
                        if (side1 == -1 && side2 == -1)
                            pointInArea = true;
                    }
                    // if angle is >= 180
                    else
                    {
                        //Debug.trace("angle type is 1")
                        if (side1 == -1 || side2 == -1)
                            pointInArea = true;
                    }
                }
                // if funnel right
                else
                {
                    // if angle is < 180
                    if (angleType == 1)
                    {
                        if (side1 == 1 && side2 == 1)
                            pointInArea = true;
                    }
                    // if angle is >= 180
                    else
                    {
                        if (side1 == 1 || side2 == 1)
                            pointInArea = true;
                    }
                }
                if (pointInArea)
                {
                    // encirclePoints.splice(index, 0);
                    encirclePoints.Insert(index, new Point2D(xToCheck, yToCheck));
                    index++;
                }
                else
                    index = 0;
            }
            if (side == -1)
                encirclePoints.Reverse();
        }
    }
}
