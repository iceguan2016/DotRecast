
using System.Collections.Generic;
using System.Drawing;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Util;

namespace Pathfinding.Triangulation.Math
{
    public class Intersection
    {
        public static Intersection EVertex(Vertex vertex)
        {
            return new Intersection_EVertex(vertex);
        }


        public static Intersection EEdge(Edge edge)
        {
            return new Intersection_EEdge(edge);
        }


        public static Intersection EFace(Face face)
        {
            return new Intersection_EFace(face);
        }
    }

    public sealed class Intersection_EVertex : Intersection
    {

        public Intersection_EVertex(Vertex vertex)
        {
            this.vertex = vertex;
        }

        public override int GetHashCode()
        {
            return this.vertex.GetHashCode();
        }

        public override bool Equals(object other)
        {
            if (System.Object.ReferenceEquals(((object)(this)), ((object)(other))))
            {
                return true;
            }

            var en = (other as Intersection_EVertex);
            if ((en == null))
            {
                return false;
            }

            if (!this.vertex.Equals(en.vertex))
            {
                return false;
            }
            return true;
        }


        public string toString()
        {
            return "EVertex " + this.vertex.toString();
        }


        public readonly Vertex vertex;

    }

    public sealed class Intersection_EEdge : Intersection
    {

        public Intersection_EEdge(Edge edge)
        {
            this.edge = edge;
        }

        public override int GetHashCode()
        {
            return this.edge.GetHashCode();
        }

        public override bool Equals(object other)
        {
            if (System.Object.ReferenceEquals(((object)(this)), ((object)(other))))
            {
                return true;
            }

            var en = (other as Intersection_EEdge);
            if ((en == null))
            {
                return false;
            }

            if (!this.edge.Equals(en.edge))
            {
                return false;
            }
            return true;
        }


        public string toString()
        {
            return "EEdge " + this.edge.toString();
        }


        public readonly Edge edge;

    }

    public sealed class Intersection_EFace : Intersection
    {

        public Intersection_EFace(Face face)
        {
            this.face = face;
        }

        public override int GetHashCode()
        {
            return this.face.GetHashCode();
        }

        public override bool Equals(object other)
        {
            if (System.Object.ReferenceEquals(((object)(this)), ((object)(other))))
            {
                return true;
            }

            var en = (other as Intersection_EFace);
            if ((en == null))
            {
                return false;
            }

            if (!this.face.Equals(en.face))
            {
                return false;
            }
            return true;
        }


        public string toString()
        {
            return "EFace " + this.face.toString();
        }


        public readonly Face face;

    }

    public class Geom2D
    {
        static Geom2D()
        {
            Geom2D.__samples = new List<Vertex>();
            Geom2D.__circumcenter = FixMath.F64Vec2.Zero;
        }


        public Geom2D()
        {
        }


        public static RandGenerator _randGen;

        public static List<Vertex> __samples;

        public static FixMath.F64Vec2 __circumcenter;

        // return one the following, in priority order:
        // - an existant vertex (if (x, y) lies on this vertex)
        // or
        // - an existant edge (if (x, y) lies on this edge )
        // or
        // - an existant face (if (x, y) lies on this face )
        // or
        // - null if outside mesh
        // YOU SHOULD USE THIS FUNCTION ONLY FOR COORDINATES INSIDE SAFE AREA
        public static Intersection locatePosition(FixMath.F64 x, FixMath.F64 y, Mesh mesh)
        {
            // jump and walk algorithm

            if (_randGen == null)
                _randGen = new RandGenerator();
            _randGen.set_seed(FixMath.F64.FloorToInt(x * 10 + 4 * y));

            int i;

            __samples.Clear();
            var numSamples = FixMath.F64.FloorToInt(FixMath.F64.PowFast(FixMath.F64.FromInt(mesh._vertices.Count), FixMath.F64.One / FixMath.F64.FromInt(3)));
            _randGen._rangeMin = 0;
            _randGen._rangeMax = mesh._vertices.Count - 1;
            for (i = 0; i < numSamples; ++i)
            {
                var _rnd = _randGen.next();

                // Debug.assertFalse(_rnd< 0 || _rnd> mesh._vertices.length - 1, '_rnd: $_rnd');
                // Debug.assertFalse(mesh._vertices == null, 'vertices: ${mesh._vertices.length}');
                __samples.Add(mesh._vertices[_rnd]);
            }

            Vertex currVertex;
            FixMath.F64Vec2 currVertexPos;
            FixMath.F64 distSquared;
            FixMath.F64 minDistSquared = FixMath.F64.MaxValue;
            Vertex closedVertex = null;
            for (i = 0; i < numSamples; ++i)
            {
                currVertex = __samples[i];
                currVertexPos = currVertex._pos;
                distSquared = (currVertexPos.X - x) * (currVertexPos.X - x) + (currVertexPos.Y - y) * (currVertexPos.Y - y);
                if (distSquared < minDistSquared)
                {
                    minDistSquared = distSquared;
                    closedVertex = currVertex;
                }
            }

            Face currFace;
            var iterFace = new FromVertexToHoldingFaces();
            iterFace.set_fromVertex(closedVertex);
            currFace = iterFace.next();

            var faceVisited = new HashSet<Face>();
            Edge currEdge;
            var iterEdge = new FromFaceToInnerEdges();
            Intersection objectContainer = null;
            int relativPos;
            int numIter = 0;
            //while ( faceVisited[ currFace ] || !(objectContainer = isInFace(x, y, currFace)) )
            while (true)
            {
                bool tmp = false;
                if (!faceVisited.Contains(currFace))
                {
                    objectContainer = Geom2D.isInFace(x, y, currFace);
                    tmp = objectContainer == null;
                }
                else
                {
                    tmp = true;
                }

                if (!tmp)
                {
                    break;
                }

                faceVisited.Add(currFace);

                numIter++;
                if (numIter == 50)
                {
                    Debug.LogError("WALK TAKE MORE THAN 50 LOOP");
                }

                // 迭代face的innerEdge找到(x, y)在该edge右侧的方向才停止
                // 下一个face就从该右侧face继续遍历
                iterEdge.set_fromFace(currFace);
                do
                {
                    currEdge = iterEdge.next();
                    if (currEdge == null)
                    {
                        Debug.LogError("KILL PATH");
                        return null;
                    }
                    relativPos = getRelativePosition(x, y, currEdge);
                } while ((relativPos == 1 || relativPos == 0));

                currFace = currEdge.get_rightFace();
            }

            return objectContainer;
        }

        public static Face nearestFace(FixMath.F64 x, FixMath.F64 y, Mesh mesh)
        {
            var loc = locatePosition(x, y, mesh);
            return null;
        }

        public static FixMath.F64Vec2 closestPointOnSegment(FixMath.F64Vec2 point, FixMath.F64Vec2 start, FixMath.F64Vec2 end)
        {
            var ab = end - start;
            var ap = point - start;
            var dot_product_ab_ap = FixMath.F64Vec2.Dot(ab, ap);
            var square_length_ab = FixMath.F64Vec2.LengthSqr(ab);
            var t = dot_product_ab_ap / FixMath.F64.Max(square_length_ab, FixMath.F64.Epsilon);

            if (t <= FixMath.F64.Zero)
            {
                return start; // 最近点是A
            }
            else if (t >= FixMath.F64.One)
            {
                return end; // 最近点是B
            }
            else
            {
                // 最近点在线段AB之间
                return FixMath.F64Vec2.Lerp(start, end, t);
            }
        }

        public static FixMath.F64Vec2 closestPointToFace(FixMath.F64 x, FixMath.F64 y, Face face)
        {
            var point = new FixMath.F64Vec2(x, y);
            if (isInFace(point.X, point.Y, face) != null)
            {
                // 点在三角形内部
                return point;
            }

            var minDist = FixMath.F64.MaxValue;
            var closestPoint = point;
            FromFaceToInnerEdges iterEdge = new FromFaceToInnerEdges();
            iterEdge.set_fromFace(face);
            while (true)
            {
                var innerEdge = iterEdge.next();
                if (innerEdge == null)
                {
                    break;
                }

                var v0 = innerEdge.get_originVertex();
                var v1 = innerEdge.get_destinationVertex();

                var closest = closestPointOnSegment(point, v0.get_pos(), v1.get_pos());
                var dist = FixMath.F64Vec2.LengthSqr(point - closest);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = closest;
                }
            }
            return closestPoint;
        }

        public static bool isCircleIntersectingAnyConstraint(FixMath.F64 x, FixMath.F64 y, FixMath.F64 radius, Mesh mesh)
        {
            if (x <= mesh._xmin || x >= mesh._xmax || y <= mesh._ymin || y >= mesh._ymax)
                return true;

            var loc = Geom2D.locatePosition(x, y, mesh);
            Face face = null;
            if (loc is Intersection_EVertex interVertex)
            {
                face = interVertex.vertex._edge._leftFace;
            }
            else if (loc is Intersection_EEdge interEdge)
            {
                face = interEdge.edge._leftFace;
            }
            else if (loc is Intersection_EFace interFace)
            {
                face = interFace.face;
            }

            // if a vertex is in the circle, a contrainst must intersect the circle
            // because a vertex always belongs to a contrained edge
            var radiusSquared = radius * radius;
            FixMath.F64Vec2 pos;
            FixMath.F64 distSquared;
            pos = face._edge._originVertex._pos;
            distSquared = (pos.X - x) * (pos.X - x) + (pos.Y - y) * (pos.Y - y);
            if (distSquared <= radiusSquared)
            {
                return true;
            }
            pos = face._edge._nextLeftEdge._originVertex._pos;
            distSquared = (pos.X - x) * (pos.X - x) + (pos.Y - y) * (pos.Y - y);
            if (distSquared <= radiusSquared)
            {
                return true;
            }
            pos = face._edge._nextLeftEdge._nextLeftEdge._originVertex._pos;
            distSquared = (pos.X - x) * (pos.X - x) + (pos.Y - y) * (pos.Y - y);
            if (distSquared <= radiusSquared)
            {
                return true;
            }  // check if edge intersects  



            var edgesToCheck = new Queue<Edge>();
            edgesToCheck.Enqueue(face._edge);
            edgesToCheck.Enqueue(face._edge._nextLeftEdge);
            edgesToCheck.Enqueue(face._edge._nextLeftEdge._nextLeftEdge);

            Edge edge = null;
            FixMath.F64Vec2 pos1;
            FixMath.F64Vec2 pos2;
            var checkedEdges = new HashSet<Edge>();
            bool intersecting;
            while (edgesToCheck.Count > 0)
            {
                edge = edgesToCheck.Dequeue();
                checkedEdges.Add(edge);
                pos1 = edge.get_originVertex().get_pos();
                pos2 = edge.get_destinationVertex().get_pos();
                intersecting = intersectionsSegmentCircle(pos1.X, pos1.Y, pos2.X, pos2.Y, x, y, radius);
                if (intersecting)
                {
                    if (edge.get_isConstrained())
                    {
                        return true;
                    }
                    else
                    {
                        edge = edge.get_oppositeEdge().get_nextLeftEdge();
                        if (!checkedEdges.Contains(edge) && !checkedEdges.Contains(edge.get_oppositeEdge()) &&
                            !edgesToCheck.Contains(edge) && !edgesToCheck.Contains(edge.get_oppositeEdge()))
                        {
                            edgesToCheck.Enqueue(edge);
                        }
                        edge = edge.get_nextLeftEdge();
                        if (!checkedEdges.Contains(edge) && !checkedEdges.Contains(edge.get_oppositeEdge()) &&
                            !edgesToCheck.Contains(edge) && !edgesToCheck.Contains(edge.get_oppositeEdge()))
                        {
                            edgesToCheck.Enqueue(edge);
                        }
                    }
                }
            }

            return false;
        }

        // return the relative direction from (x1,y1), to (x3,y3) through (x2, y2)
        // the function returns:
        // 0 if the path is a straight line
        // 1 if the path goes to the left
        // -1 if the path goes to the right
        public static int getDirection(FixMath.F64 x1, FixMath.F64 y1, FixMath.F64 x2, FixMath.F64 y2, FixMath.F64 x3, FixMath.F64 y3)
        {
            // dot product with the orthogonal vector pointing left vector of eUp:
            var dot = (x3 - x1) * (y2 - y1) + (y3 - y1) * (-x2 + x1);

            // check sign
            return (FixMath.F64.Abs(dot) <= Constants.EPSILON) ? 0 : (dot > 0 ? 1 : -1);
        }

        // second version of getDirection. More accurate and safer version
        // return the relative direction from (x1,y1), to (x3,y3) through (x2, y2)
        // the function returns:
        // 0 if the path is a straight line
        // 1 if the path goes to the left
        // -1 if the path goes to the right
        public static int getDirection2(FixMath.F64 x1, FixMath.F64 y1, FixMath.F64 x2, FixMath.F64 y2, FixMath.F64 x3, FixMath.F64 y3)
        {
            // dot product with the orthogonal vector pointing left vector of eUp:
            var dot = (x3 - x1) * (y2 - y1) + (y3 - y1) * (-x2 + x1);

            // check sign
            if (FixMath.F64.Abs(dot) <= Constants.EPSILON)
            {
                return 0;
            }
            else if (dot > 0)
            {
                if (distanceSquaredPointToLine(x3, y3, x1, y1, x2, y2) <= Constants.EPSILON_SQUARED)
                {
                    return 0;
                }
                else
                {
                    return 1;
                }
            }
            else
            {
                if (distanceSquaredPointToLine(x3, y3, x1, y1, x2, y2) <= Constants.EPSILON_SQUARED)
                {
                    return 0;
                }
                else
                {
                    return -1;
                }
            }
            return 0;
        }

        // eUp seen as an infinite line splits the 2D space in 2 parts (left and right),
        // the function returns:
        //   0 if the (x, y) lies on the line
        //   1 if the (x, y) lies at left
        //   -1 if the (x, y) lies at right
        public static int getRelativePosition(FixMath.F64 x, FixMath.F64 y, Edge eUp)
        {
            var originPos = eUp.get_originVertex().get_pos();
            var destPos = eUp.get_destinationVertex().get_pos();
            return Geom2D.getDirection(originPos.X, originPos.Y, destPos.X, destPos.Y, x, y);
        }


        public static int getRelativePosition2(FixMath.F64 x, FixMath.F64 y, Edge eUp)
        {
            var originPos = eUp.get_originVertex().get_pos();
            var destPos = eUp.get_destinationVertex().get_pos();
            return Geom2D.getDirection2(originPos.X, originPos.Y, destPos.X, destPos.Y, x, y);
        }

        // squared distance from point p to infinite line (a, b)
        public static FixMath.F64 distanceSquaredPointToLine(FixMath.F64 px, FixMath.F64 py, FixMath.F64 ax, FixMath.F64 ay, FixMath.F64 bx, FixMath.F64 by)
        {
            var a_b_squaredLength = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
            var dotProduct = (px - ax) * (bx - ax) + (py - ay) * (by - ay);
            var p_a_squaredLength = (ax - px) * (ax - px) + (ay - py) * (ay - py);
            return p_a_squaredLength - dotProduct * dotProduct / a_b_squaredLength;
        }

        // squared distance from point p to finite segment [a, b]
        public static FixMath.F64 distanceSquaredPointToSegment(FixMath.F64 px, FixMath.F64 py, FixMath.F64 ax, FixMath.F64 ay, FixMath.F64 bx, FixMath.F64 by)
        {
            var a_b_squaredLength = (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
            var dotProduct = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / a_b_squaredLength;
            if (dotProduct < 0)
            {
                return (px - ax) * (px - ax) + (py - ay) * (py - ay);
            }
            else if (dotProduct <= 1)
            {
                var p_a_squaredLength = (ax - px) * (ax - px) + (ay - py) * (ay - py);
                return p_a_squaredLength - dotProduct * dotProduct * a_b_squaredLength;
            }
            else
            {
                return (px - bx) * (px - bx) + (py - by) * (py - by);
            }
        }

        public static FixMath.F64 distanceSquaredVertexToEdge(Vertex vertex, Edge edge)
        {
            return distanceSquaredPointToSegment(vertex._pos.X, vertex._pos.Y, edge._originVertex._pos.X, edge._originVertex._pos.Y, edge.get_destinationVertex()._pos.X, edge.get_destinationVertex()._pos.Y);
        }

        // the function checks by priority:
        // - if the (x, y) lies on a vertex of the polygon, it will return this vertex
        // - if the (x, y) lies on a edge of the polygon, it will return this edge
        // - if the (x, y) lies inside the polygon, it will return the polygon
        // - if the (x, y) lies outside the polygon, it will return null
        public static Intersection isInFace(FixMath.F64 x, FixMath.F64 y, Face polygon)
        {
            // remember polygons are triangle only,
            // and we suppose we have not degenerated flat polygons !

            Intersection result = null;

            var e1_2 = polygon._edge;
            var e2_3 = e1_2._nextLeftEdge;
            var e3_1 = e2_3._nextLeftEdge;
            if (getRelativePosition(x, y, e1_2) >= 0 && getRelativePosition(x, y, e2_3) >= 0 && getRelativePosition(x, y, e3_1) >= 0)
            {
                var v1 = e1_2._originVertex;
                var v2 = e2_3._originVertex;
                var v3 = e3_1._originVertex;

                var x1 = v1._pos.X;
                var y1 = v1._pos.Y;
                var x2 = v2._pos.X;
                var y2 = v2._pos.Y;
                var x3 = v3._pos.X;
                var y3 = v3._pos.Y;

                var v_v1squaredLength = (x1 - x) * (x1 - x) + (y1 - y) * (y1 - y);
                var v_v2squaredLength = (x2 - x) * (x2 - x) + (y2 - y) * (y2 - y);
                var v_v3squaredLength = (x3 - x) * (x3 - x) + (y3 - y) * (y3 - y);
                var v1_v2squaredLength = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
                var v2_v3squaredLength = (x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2);
                var v3_v1squaredLength = (x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3);

                var dot_v_v1v2 = (x - x1) * (x2 - x1) + (y - y1) * (y2 - y1);
                var dot_v_v2v3 = (x - x2) * (x3 - x2) + (y - y2) * (y3 - y2);
                var dot_v_v3v1 = (x - x3) * (x1 - x3) + (y - y3) * (y1 - y3);

                var v_e1_2squaredLength = v_v1squaredLength - dot_v_v1v2 * dot_v_v1v2 / v1_v2squaredLength;
                var v_e2_3squaredLength = v_v2squaredLength - dot_v_v2v3 * dot_v_v2v3 / v2_v3squaredLength;
                var v_e3_1squaredLength = v_v3squaredLength - dot_v_v3v1 * dot_v_v3v1 / v3_v1squaredLength;

                var closeTo_e1_2 = v_e1_2squaredLength <= Constants.EPSILON_SQUARED;
                var closeTo_e2_3 = v_e2_3squaredLength <= Constants.EPSILON_SQUARED;
                var closeTo_e3_1 = v_e3_1squaredLength <= Constants.EPSILON_SQUARED;

                if (closeTo_e1_2)
                {
                    if (closeTo_e3_1)
                    {
                        result = Intersection.EVertex(v1);
                    }
                    else if (closeTo_e2_3)
                    {
                        result = Intersection.EVertex(v2);
                    }
                    else
                    {
                        result = Intersection.EEdge(e1_2);
                    }
                }
                else if (closeTo_e2_3)
                {
                    if (closeTo_e3_1)
                    {
                        result = Intersection.EVertex(v3);
                    }
                    else
                    {
                        result = Intersection.EEdge(e2_3);
                    }
                }
                else if (closeTo_e3_1)
                {
                    result = Intersection.EEdge(e3_1);
                }
                else
                {
                    result = Intersection.EFace(polygon);
                }
            }

            return result;
        }

        // 
        public static bool intersectionsSegmentCircle(
            FixMath.F64 p0x, FixMath.F64 p0y, FixMath.F64 p1x, FixMath.F64 p1y, FixMath.F64 cx, FixMath.F64 cy, FixMath.F64 r, List<FixMath.F64> result = null)
        {
            var p0xSQD = (p0x * p0x);
            var p0ySQD = (p0y * p0y);
            var a = ((((((p1y * p1y) - ((2 * p1y) * p0y)) + p0ySQD) + (p1x * p1x)) - ((2 * p1x) * p0x)) + p0xSQD);
            var b = (((((((((2 * p0y) * cy) - (2 * p0xSQD)) + ((2 * p1y) * p0y)) - (2 * p0ySQD)) + ((2 * p1x) * p0x)) - ((2 * p1x) * cx)) + ((2 * p0x) * cx)) - ((2 * p1y) * cy));
            var c = ((((((p0ySQD + (cy * cy)) + (cx * cx)) - ((2 * p0y) * cy)) - ((2 * p0x) * cx)) + p0xSQD) - (r * r));
            var delta = ((b * b) - ((4 * a) * c));
            var deltaSQRT = FixMath.F64.Zero;
            var t0 = FixMath.F64.Zero;
            var t1 = FixMath.F64.Zero;
            if ((delta < 0))
            {
                return false;
            }
            else if ((delta == 0))
            {
                t0 = (-(b) / ((2 * a)));
                if (((t0 < 0) || (t0 > 1)))
                {
                    return false;
                }

                if ((result != null))
                {
                    //  [intersect0.x, intersect0.y, t0]    // we return a 3 elements array, under the form
                    {
                        var f = (p0x + (t0 * ((p1x - p0x))));
                        result.Add(f);
                    }

                    {
                        var f1 = (p0y + (t0 * ((p1y - p0y))));
                        result.Add(f1);
                    }

                    {
                        var f2 = t0;
                        result.Add(f2);
                    }

                }

                return true;
            }
            else
            {
                deltaSQRT = FixMath.F64.Sqrt(delta);
                t0 = (((-(b) + deltaSQRT)) / ((2 * a)));
                t1 = (((-(b) - deltaSQRT)) / ((2 * a)));
                bool intersecting = false;
                // we return a n elements array, under the form:
                //  [intersect0.x, intersect0.y, t0
                //    , intersect1.x, intersect1.y, t1]
                if (((0 <= t0) && (t0 <= 1)))
                {
                    if ((result != null))
                    {
                        {
                            var f3 = (p0x + (t0 * ((p1x - p0x))));
                            result.Add(f3);
                        }

                        {
                            var f4 = (p0y + (t0 * ((p1y - p0y))));
                            result.Add(f4);
                        }

                        {
                            var f5 = t0;
                            result.Add(f5);
                        }

                    }

                    intersecting = true;
                }

                if (((0 <= t1) && (t1 <= 1)))
                {
                    if ((result != null))
                    {
                        {
                            var f6 = (p0x + (t1 * ((p1x - p0x))));
                            result.Add(f6);
                        }

                        {
                            var f7 = (p0y + (t1 * ((p1y - p0y))));
                            result.Add(f7);
                        }

                        {
                            var f8 = t1;
                            result.Add(f8);
                        }

                    }

                    intersecting = true;
                }

                return intersecting;
            }
        }


        public static bool intersectionsLineCircle(
            FixMath.F64 p0x, FixMath.F64 p0y, FixMath.F64 p1x, FixMath.F64 p1y, FixMath.F64 cx, FixMath.F64 cy, FixMath.F64 r, List<FixMath.F64> result = null)
        {
            var p0xSQD = (p0x * p0x);
            var p0ySQD = (p0y * p0y);
            var a = ((((((p1y * p1y) - ((2 * p1y) * p0y)) + p0ySQD) + (p1x * p1x)) - ((2 * p1x) * p0x)) + p0xSQD);
            var b = (((((((((2 * p0y) * cy) - (2 * p0xSQD)) + ((2 * p1y) * p0y)) - (2 * p0ySQD)) + ((2 * p1x) * p0x)) - ((2 * p1x) * cx)) + ((2 * p0x) * cx)) - ((2 * p1y) * cy));
            var c = ((((((p0ySQD + (cy * cy)) + (cx * cx)) - ((2 * p0y) * cy)) - ((2 * p0x) * cx)) + p0xSQD) - (r * r));
            var delta = ((b * b) - ((4 * a) * c));
            var deltaSQRT = FixMath.F64.Zero;
            var t0 = FixMath.F64.Zero;
            var t1 = FixMath.F64.Zero;
            if ((delta < 0))
            {
                return false;
            }
            else if ((delta == 0))
            {
                t0 = (-(b) / ((2 * a)));
                {
                    {
                        var f = (p0x + (t0 * ((p1x - p0x))));
                        result.Add(f);
                    }

                    {
                        var f1 = (p0y + (t0 * ((p1y - p0y))));
                        result.Add(f1);
                    }

                    {
                        var f2 = t0;
                        result.Add(f2);
                    }

                }

            }
            else if ((delta > 0))
            {
                deltaSQRT = FixMath.F64.Sqrt(delta);
                t0 = (((-(b) + deltaSQRT)) / ((2 * a)));
                t1 = (((-(b) - deltaSQRT)) / ((2 * a)));
                {
                    {
                        var f3 = (p0x + (t0 * ((p1x - p0x))));
                        result.Add(f3);
                    }

                    {
                        var f4 = (p0y + (t0 * ((p1y - p0y))));
                        result.Add(f4);
                    }

                    {
                        var f5 = t0;
                        result.Add(f5);
                    }

                    {
                        var f6 = (p0x + (t1 * ((p1x - p0x))));
                        result.Add(f6);
                    }

                    {
                        var f7 = (p0y + (t1 * ((p1y - p0y))));
                        result.Add(f7);
                    }

                    {
                        var f8 = t1;
                        result.Add(f8);
                    }

                }

            }

            return true;
        }

        // paramIntersection: [t1, t2], (0 <= t1 <= 1, 0 <= t2 <= 1)
        public static bool intersections2segments(
            FixMath.F64 s1p1x, FixMath.F64 s1p1y, FixMath.F64 s1p2x, FixMath.F64 s1p2y,
            FixMath.F64 s2p1x, FixMath.F64 s2p1y, FixMath.F64 s2p2x, FixMath.F64 s2p2y,
            out FixMath.F64Vec2 posIntersection, List<FixMath.F64> paramIntersection = null, bool infiniteLineMode = false)
        {
            posIntersection = FixMath.F64Vec2.Zero;
            var t1 = FixMath.F64.Zero;
            var t2 = FixMath.F64.Zero;
            bool result = false;
            var divisor = ((((s1p1x - s1p2x)) * ((s2p1y - s2p2y))) + (((s1p2y - s1p1y)) * ((s2p1x - s2p2x))));
            if ((divisor == 0))
            {
                result = false;
            }
            else
            {
                result = true;
                if (!infiniteLineMode || paramIntersection != null)
                {
                    // if we consider edges as finite segments, we must check t1 and t2 values
                    t1 = ((((((s1p1x * ((s2p1y - s2p2y))) + (s1p1y * ((s2p2x - s2p1x)))) + (s2p1x * s2p2y)) - (s2p1y * s2p2x))) / divisor);
                    t2 = ((((((s1p1x * ((s2p1y - s1p2y))) + (s1p1y * ((s1p2x - s2p1x)))) - (s1p2x * s2p1y)) + (s1p2y * s2p1x))) / divisor);
                    if ((!(infiniteLineMode) && !((((((0 <= t1) && (t1 <= 1)) && (0 <= t2)) && (t2 <= 1))))))
                    {
                        result = false;
                    }

                }

            }

            if (result)
            {
                posIntersection.X = (s1p1x + (t1 * ((s1p2x - s1p1x))));
                posIntersection.Y = (s1p1y + (t1 * ((s1p2y - s1p1y))));

                if ((paramIntersection != null))
                {
                    paramIntersection.Add(t1);
                    paramIntersection.Add(t2);
                }

            }

            return result;
        }


        public static bool intersections2edges(Edge edge1, Edge edge2, out FixMath.F64Vec2 posIntersection, List<FixMath.F64> paramIntersection = null, bool infiniteLineMode = false)
        {
            var originPos1 = edge1.get_originVertex().get_pos();
            var destPos1 = edge1.get_destinationVertex().get_pos();
            var originPos2 = edge2.get_originVertex().get_pos();
            var destPos2 = edge2.get_destinationVertex().get_pos();
            return Geom2D.intersections2segments(originPos1.X, originPos1.Y, destPos1.X, destPos1.Y, originPos2.X, originPos2.Y, destPos2.X, destPos2.Y, out posIntersection, paramIntersection, infiniteLineMode);
        }

        // return:
        // - true if the segment is totally or partially in the triangle
        // - false if the segment is totally outside the triangle
        public static bool clipSegmentByTriangle(
            FixMath.F64 s1x, FixMath.F64 s1y, FixMath.F64 s2x, FixMath.F64 s2y,
            FixMath.F64 t1x, FixMath.F64 t1y, FixMath.F64 t2x, FixMath.F64 t2y, FixMath.F64 t3x, FixMath.F64 t3y,
            out FixMath.F64Vec2 pResult1, out FixMath.F64Vec2 pResult2)
        {
            pResult1 = FixMath.F64Vec2.Zero;
            pResult2 = FixMath.F64Vec2.Zero;
            int side1_1;
            int side1_2;
            side1_1 = getDirection(t1x, t1y, t2x, t2y, s1x, s1y);
            side1_2 = getDirection(t1x, t1y, t2x, t2y, s2x, s2y);
            // if both segment points are on right side
            if (side1_1 <= 0 && side1_2 <= 0)
                return false;

            int side2_1;
            int side2_2;
            side2_1 = getDirection(t2x, t2y, t3x, t3y, s1x, s1y);
            side2_2 = getDirection(t2x, t2y, t3x, t3y, s2x, s2y);
            // if both segment points are on right side
            if (side2_1 <= 0 && side2_2 <= 0)
                return false;

            int side3_1;
            int side3_2;
            side3_1 = getDirection(t3x, t3y, t1x, t1y, s1x, s1y);
            side3_2 = getDirection(t3x, t3y, t1x, t1y, s2x, s2y);
            // if both segment points are on right side
            if (side3_1 <= 0 && side3_2 <= 0)
                return false;

            // both segment points are in triangle  ;
            if ((side1_1 >= 0 && side2_1 >= 0 && side3_1 >= 0) && (side1_2 >= 0 && side2_2 >= 0 && side3_2 >= 0))
            {
                pResult1.X = s1x;
                pResult1.Y = s1y;
                pResult2.X = s2x;
                pResult2.Y = s2y;
                return true;
            }

            int n = 0;
            // check intersection between segment and 1st side triangle
            if (intersections2segments(s1x, s1y, s2x, s2y, t1x, t1y, t2x, t2y, out pResult1, null))
            {
                n++;
            }  // if no intersection with 1st side triangle  



            if (n == 0)
            {
                // check intersection between segment and 1st side triangle
                if (intersections2segments(s1x, s1y, s2x, s2y, t2x, t2y, t3x, t3y, out pResult1, null))
                {
                    n++;
                }
            }
            else
            {
                if (intersections2segments(s1x, s1y, s2x, s2y, t2x, t2y, t3x, t3y, out pResult2, null))
                {
                    // we check if the segment is not on t2 triangle point
                    if (-Constants.EPSILON > (pResult1.X - pResult2.X) || (pResult1.X - pResult2.X) > Constants.EPSILON || -Constants.EPSILON > (pResult1.Y - pResult2.Y) || (pResult1.Y - pResult2.Y) > Constants.EPSILON)
                    {
                        n++;
                    }
                }
            }  // if intersection neither 1st nor 2nd side triangle  



            if (n == 0)
            {
                if (intersections2segments(s1x, s1y, s2x, s2y, t3x, t3y, t1x, t1y, out pResult1, null))
                {
                    n++;
                }
            }
            // if one intersection, we identify the segment point in the triangle
            else if (n == 1)
            {
                if (intersections2segments(s1x, s1y, s2x, s2y, t3x, t3y, t1x, t1y, out pResult2, null))
                {
                    if (-Constants.EPSILON > (pResult1.X - pResult2.X) || (pResult1.X - pResult2.X) > Constants.EPSILON || -Constants.EPSILON > (pResult1.Y - pResult2.Y) || (pResult1.Y - pResult2.Y) > Constants.EPSILON)
                    {
                        n++;
                    }
                }
            }



            if (n == 1)
            {
                if (side1_1 >= 0 && side2_1 >= 0 && side3_1 >= 0)
                {
                    pResult2.X = s1x;
                    pResult2.Y = s1y;
                }
                else if (side1_2 >= 0 && side2_2 >= 0 && side3_2 >= 0)
                {
                    pResult2.X = s2x;
                    pResult2.Y = s2y;
                }
                else
                {
                    // 1 intersection and none point in triangle : degenerate case
                    n = 0;
                }
            }

            if (n > 0)
                return true;
            else
                return false;
        }

        public static bool isSegmentIntersectingTriangle(
            FixMath.F64 s1x, FixMath.F64 s1y, FixMath.F64 s2x, FixMath.F64 s2y,
            FixMath.F64 t1x, FixMath.F64 t1y, FixMath.F64 t2x, FixMath.F64 t2y, FixMath.F64 t3x, FixMath.F64 t3y)
        {
            int side1_1 = Geom2D.getDirection(t1x, t1y, t2x, t2y, s1x, s1y);
            int side1_2 = Geom2D.getDirection(t1x, t1y, t2x, t2y, s2x, s2y);
            if (((side1_1 <= 0) && (side1_2 <= 0)))
            {
                return false;
            }

            int side2_1 = Geom2D.getDirection(t2x, t2y, t3x, t3y, s1x, s1y);
            int side2_2 = Geom2D.getDirection(t2x, t2y, t3x, t3y, s2x, s2y);
            if (((side2_1 <= 0) && (side2_2 <= 0)))
            {
                return false;
            }

            int side3_1 = Geom2D.getDirection(t3x, t3y, t1x, t1y, s1x, s1y);
            int side3_2 = Geom2D.getDirection(t3x, t3y, t1x, t1y, s2x, s2y);
            if (((side3_1 <= 0) && (side3_2 <= 0)))
            {
                return false;
            }

            if ((((side1_1 == 1) && (side2_1 == 1)) && (side3_1 == 1)))
            {
                return true;
            }

            if ((((side1_1 == 1) && (side2_1 == 1)) && (side3_1 == 1)))
            {
                return true;
            }

            int side1 = default(int);
            int side2 = default(int);
            if ((((side1_1 == 1) && (side1_2 <= 0)) || ((side1_1 <= 0) && (side1_2 == 1))))
            {
                side1 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t1x, t1y);
                side2 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t2x, t2y);
                if ((((side1 == 1) && (side2 <= 0)) || ((side1 <= 0) && (side2 == 1))))
                {
                    return true;
                }

            }

            if ((((side2_1 == 1) && (side2_2 <= 0)) || ((side2_1 <= 0) && (side2_2 == 1))))
            {
                side1 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t2x, t2y);
                side2 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t3x, t3y);
                if ((((side1 == 1) && (side2 <= 0)) || ((side1 <= 0) && (side2 == 1))))
                {
                    return true;
                }

            }

            if ((((side3_1 == 1) && (side3_2 <= 0)) || ((side3_1 <= 0) && (side3_2 == 1))))
            {
                side1 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t3x, t3y);
                side2 = Geom2D.getDirection(s1x, s1y, s2x, s2y, t1x, t1y);
                if ((((side1 == 1) && (side2 <= 0)) || ((side1 <= 0) && (side2 == 1))))
                {
                    return true;
                }

            }

            return false;
        }

        public static bool isDelaunay(Edge edge)
        {
            // 取edge的对角线上2个顶点vCorner和vOpposite
            var vLeft = edge.get_originVertex();
            var vRight = edge.get_destinationVertex();
            var vCorner = edge.get_nextLeftEdge().get_destinationVertex();
            var vOpposite = edge.get_nextRightEdge().get_destinationVertex();
            Geom2D.getCircumcenter(vCorner.get_pos().X, vCorner.get_pos().Y, vLeft.get_pos().X, vLeft.get_pos().Y, vRight.get_pos().X, vRight.get_pos().Y, out __circumcenter);
            var squaredRadius = ((((vCorner.get_pos().X - Geom2D.__circumcenter.X)) * ((vCorner.get_pos().X - Geom2D.__circumcenter.X))) + (((vCorner.get_pos().Y - Geom2D.__circumcenter.Y)) * ((vCorner.get_pos().Y - Geom2D.__circumcenter.Y))));
            var squaredDistance = ((((vOpposite.get_pos().X - Geom2D.__circumcenter.X)) * ((vOpposite.get_pos().X - Geom2D.__circumcenter.X))) + (((vOpposite.get_pos().Y - Geom2D.__circumcenter.Y)) * ((vOpposite.get_pos().Y - Geom2D.__circumcenter.Y))));
            return (squaredDistance >= (squaredRadius + Constants.EPSILON_SQUARED));
        }

        public static FixMath.F64Vec2 getCircumcenter(
            FixMath.F64 x1, FixMath.F64 y1, FixMath.F64 x2, FixMath.F64 y2, FixMath.F64 x3, FixMath.F64 y3, out FixMath.F64Vec2 result)
        {
            result = FixMath.F64Vec2.Zero;

            var m1 = (((x1 + x2)) / 2);
            var m2 = (((y1 + y2)) / 2);
            var m3 = (((x1 + x3)) / 2);
            var m4 = (((y1 + y3)) / 2);
            var t1 = (((((m1 * ((x1 - x3))) + (((m2 - m4)) * ((y1 - y3)))) + (m3 * ((x3 - x1))))) / ((((x1 * ((y3 - y2))) + (x2 * ((y1 - y3)))) + (x3 * ((y2 - y1))))));
            result.X = (m1 + (t1 * ((y2 - y1))));
            result.Y = (m2 - (t1 * ((x2 - x1))));
            return result;
        }

        // a edge is convex if the polygon formed by the 2 faces at left and right of this edge is convex
        public static bool isConvex(Edge edge)
        {
            bool result = true;
            Edge eLeft = edge.get_nextLeftEdge().get_oppositeEdge();
            Vertex vRight = edge.get_nextRightEdge().get_destinationVertex();
            if ((Geom2D.getRelativePosition(vRight.get_pos().X, vRight.get_pos().Y, eLeft) != -1))
            {
                result = false;
            }
            else
            {
                eLeft = edge.get_prevRightEdge();
                vRight = edge.get_prevLeftEdge().get_originVertex();
                if ((Geom2D.getRelativePosition(vRight.get_pos().X, vRight.get_pos().Y, eLeft) != -1))
                {
                    result = false;
                }

            }

            return result;
        }

        public static void projectOrthogonaly(ref FixMath.F64Vec2 vertexPos, Edge edge)
        {
            // parametric expression of edge
            // x(t1) = edge.originVertex.pos.x + t1*(edge.destinationVertex.pos.x - edge.originVertex.pos.x)
            // y(t1) = edge.originVertex.pos.y + t1*(edge.destinationVertex.pos.y - edge.originVertex.pos.y)

            // parametric expression of the segment orthogonal to edge and lying by vertex
            // x(t2) = vertexPos.x + t2*(edge.destinationVertex.pos.y - edge.originVertex.pos.y)
            // y(t2) = vertexPos.y - t2*(edge.destinationVertex.pos.x - edge.originVertex.pos.x)

            // the orthogonal projection of vertex on edge will lead to:
            // x(t1) = x(t2)
            // y(t1) = y(t2)

            // set alias letters
            var a = edge.get_originVertex()._pos.X;
            var b = edge.get_originVertex()._pos.Y;
            var c = edge.get_destinationVertex()._pos.X;
            var d = edge.get_destinationVertex()._pos.Y;
            var e = vertexPos.X;
            var f = vertexPos.Y;

            // system to solve:
            // a + t1 (c - a) = e + t2 (d - b)
            // b + t1 (d - b) = f - t2 (c - a)

            // solution:
            var t1 = (a * a - a * c - a * e + b * b - b * d - b * f + c * e + d * f) / (a * a - 2 * a * c + b * b - 2 * b * d + c * c + d * d);

            // set position:
            vertexPos.X = a + t1 * (c - a);
            vertexPos.Y = b + t1 * (d - b);
        }

        // fill the result vector with 4 elements, with the form:
        // [intersect0.x, intersect0.y, intersect1.x, intersect1.y]
        // empty if no intersection
        public static bool intersections2Circles(
            FixMath.F64 cx1, FixMath.F64 cy1, FixMath.F64 r1, FixMath.F64 cx2, FixMath.F64 cy2, FixMath.F64 r2, List<FixMath.F64> result = null)
        {
            var distRadiusSQRD = ((((cx2 - cx1)) * ((cx2 - cx1))) + (((cy2 - cy1)) * ((cy2 - cy1))));
            if ((((((cx1 != cx2) || (cy1 != cy2))) && (distRadiusSQRD <= (((r1 + r2)) * ((r1 + r2))))) && (distRadiusSQRD >= (((r1 - r2)) * ((r1 - r2))))))
            {
                var transcendPart = FixMath.F64.Sqrt(((r1 + r2) * (r1 + r2) - distRadiusSQRD) * (distRadiusSQRD - (r2 - r1) * (r2 - r1)));
                var xFirstPart = ((((cx1 + cx2)) / 2) + ((((cx2 - cx1)) * (((r1 * r1) - (r2 * r2)))) / ((2 * distRadiusSQRD))));
                var yFirstPart = ((((cy1 + cy2)) / 2) + ((((cy2 - cy1)) * (((r1 * r1) - (r2 * r2)))) / ((2 * distRadiusSQRD))));
                var xFactor = (((cy2 - cy1)) / ((2 * distRadiusSQRD)));
                var yFactor = (((cx2 - cx1)) / ((2 * distRadiusSQRD)));
                if ((result != null))
                {
                    {
                        var f = (xFirstPart + (xFactor * transcendPart));
                        result.Add(f);
                    }

                    {
                        var f1 = (yFirstPart - (yFactor * transcendPart));
                        result.Add(f1);
                    }

                    {
                        var f2 = (xFirstPart - (xFactor * transcendPart));
                        result.Add(f2);
                    }

                    {
                        var f3 = (yFirstPart + (yFactor * transcendPart));
                        result.Add(f3);
                    }

                }

                return true;
            }
            else
            {
                return false;
            }
        }

        // 点到圆的2条切线
        // https://www.youtube.com/watch?v=z-YxfG42P2M
        // based on intersections2Circles method
        // fill the result vector with 4 elements, with the form:
        // [point_tangent1.x, point_tangent1.y, point_tangent2.x, point_tangent2.y]
        // empty if no tangent
        public static bool tangentsPointToCircle(FixMath.F64 px, FixMath.F64 py, FixMath.F64 cx, FixMath.F64 cy, FixMath.F64 r, List<FixMath.F64> result = null)
        {
            var c2x = (px + cx) / 2;
            var c2y = (py + cy) / 2;
            var r2 = FixMath.F64.Half * FixMath.F64.Sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));

            return intersections2Circles(c2x, c2y, r2, cx, cy, r, result);
        }

        // 与2个圆都相切的切线(交叉)
        // <!!!> CIRCLES MUST HAVE SAME RADIUS
        public static bool tangentsCrossCircleToCircle(FixMath.F64 r, FixMath.F64 c1x, FixMath.F64 c1y, FixMath.F64 c2x, FixMath.F64 c2y, List<FixMath.F64> result = null)
        {
            var distance = FixMath.F64.Sqrt((c1x - c2x) * (c1x - c2x) + (c1y - c2y) * (c1y - c2y));

            // new circle
            var radius = distance / 4;
            var centerX = c1x + (c2x - c1x) / 4;
            var centerY = c1y + (c2y - c1y) / 4;

            if (intersections2Circles(c1x, c1y, r, centerX, centerY, radius, result))
            {
                var t1x = result[0];
                var t1y = result[1];
                var t2x = result[2];
                var t2y = result[3];

                var midX = (c1x + c2x) / 2;
                var midY = (c1y + c2y) / 2;
                var dotProd = (t1x - midX) * (c2y - c1y) + (t1y - midY) * (-c2x + c1x);
                var tproj = dotProd / (distance * distance);
                var projx = midX + tproj * (c2y - c1y);
                var projy = midY - tproj * (c2x - c1x);


                var t4x = 2 * projx - t1x;
                var t4y = 2 * projy - t1y;

                var t3x = t4x + t2x - t1x;
                var t3y = t2y + t4y - t1y;

                result.Add(t3x);
                result.Add(t3y);
                result.Add(t4x);
                result.Add(t4y);

                return true;
            }
            else
            {
                // no tangent because cicles are intersecting
                return false;
            }
        }

        // 与2个圆都相切的切线(不交叉，在同侧)
        // <!!!> CIRCLES MUST HAVE SAME RADIUS
        public static void tangentsParalCircleToCircle(FixMath.F64 r, FixMath.F64 c1x, FixMath.F64 c1y, FixMath.F64 c2x, FixMath.F64 c2y, List<FixMath.F64> result)
        {
            var distance = FixMath.F64.Sqrt((c1x - c2x) * (c1x - c2x) + (c1y - c2y) * (c1y - c2y));
            var t1x = c1x + r * (c2y - c1y) / distance;
            var t1y = c1y + r * (-c2x + c1x) / distance;
            var t2x = 2 * c1x - t1x;
            var t2y = 2 * c1y - t1y;
            var t3x = t2x + c2x - c1x;
            var t3y = t2y + c2y - c1y;
            var t4x = t1x + c2x - c1x;
            var t4y = t1y + c2y - c1y;
            result.Add(t1x);
            result.Add(t1y);
            result.Add(t2x);
            result.Add(t2y);
            result.Add(t3x);
            result.Add(t3y);
            result.Add(t4x);
            result.Add(t4y);
        }
    }
}
