using System.Numerics;
using System;
using System.Collections.Generic;
using System.Linq;
using SharpSteer2.Helpers;

namespace SharpSteer2.Pathway
{
    /// <summary>
    /// A pathway made out of triangular segments
    /// </summary>
    public class TrianglePathway
        : IPathway
    {
        private readonly Triangle[] _path;

        public IEnumerable<Triangle> Triangles
        {
            get { return _path; }
        }

        private readonly PolylinePathway _centerline;
        public PolylinePathway Centerline
        {
            get
            {
                return _centerline;
            }
        }

        public TrianglePathway(IEnumerable<Triangle> path, bool cyclic = false)
        {
            _path = path.ToArray();

            //Calculate center points
            for (int i = 0; i < _path.Length; i++)
                _path[i].PointOnPath = (FixMath.F64.Two * _path[i].A + _path[i].Edge0) / FixMath.F64.Two;

            //Calculate tangents along path
            for (int i = 0; i < _path.Length; i++)
            {
                var bIndex = cyclic ? ((i + 1) % _path.Length) : Math.Min(i + 1, _path.Length - 1);

                var vectorToNextTriangle = _path[bIndex].PointOnPath - _path[i].PointOnPath;
                var l = FixMath.F64Vec3.LengthFast(vectorToNextTriangle);

                _path[i].Tangent = vectorToNextTriangle / l;

                if (FixMath.F64.Abs(l) < FixMath.F64.Epsilon)
                    _path[i].Tangent = FixMath.F64Vec3.Zero;
            }

            _centerline = new PolylinePathway(_path.Select(a => a.PointOnPath).ToArray(), FixMath.F64.FromFloat(0.1f), cyclic);
        }

        public FixMath.F64Vec3 MapPointToPath(FixMath.F64Vec3 point, out FixMath.F64Vec3 tangent, out FixMath.F64 outside)
        {
            int index;
            return MapPointToPath(point, out tangent, out outside, out index);
        }

        private FixMath.F64Vec3 MapPointToPath(FixMath.F64Vec3 point, out FixMath.F64Vec3 tangent, out FixMath.F64 outside, out int segmentIndex)
        {
            var distanceSqr = FixMath.F64.MaxValue;
            var closestPoint = FixMath.F64Vec3.Zero;
            bool inside = false;
            segmentIndex = -1;

            for (int i = 0; i < _path.Length; i++)
            {
                bool isInside;
                var p = ClosestPointOnTriangle(ref _path[i], point, out isInside);

                var normal = (point - p);
                var dSqr = FixMath.F64Vec3.LengthSqr(normal);

                if (dSqr < distanceSqr)
                {
                    distanceSqr = dSqr;
                    closestPoint = p;
                    inside = isInside;
                    segmentIndex = i;
                }

                if (isInside)
                    break;
            }

            if (segmentIndex == -1)
                throw new InvalidOperationException("Closest Path Segment Not Found (Zero Length Path?");

            tangent = _path[segmentIndex].Tangent;
            outside = FixMath.F64.SqrtFast(distanceSqr) * (inside ? -1 : 1);
            return closestPoint;
        }

        public FixMath.F64Vec3 MapPathDistanceToPoint(FixMath.F64 pathDistance)
        {
            return _centerline.MapPathDistanceToPoint(pathDistance);

            //// clip or wrap given path distance according to cyclic flag
            //if (_cyclic)
            //    pathDistance = pathDistance % _totalPathLength;
            //else
            //{
            //    if (pathDistance < 0)
            //        return _path[0].PointOnPath;
            //    if (pathDistance >= _totalPathLength)
            //        return _path[_path.Length - 1].PointOnPath;
            //}

            //// step through segments, subtracting off segment lengths until
            //// locating the segment that contains the original pathDistance.
            //// Interpolate along that segment to find 3d point value to return.
            //for (int i = 1; i < _path.Length; i++)
            //{
            //    if (_path[i].Length < pathDistance)
            //    {
            //        pathDistance -= _path[i].Length;
            //    }
            //    else
            //    {
            //        float ratio = pathDistance / _path[i].Length;

            //        var l = Vector3.Lerp(_path[i].PointOnPath, _path[i].PointOnPath + _path[i].Tangent * _path[i].Length, ratio);
            //        return l;
            //    }
            //}

            //return Vector3.Zero;
        }

        public FixMath.F64 MapPointToPathDistance(FixMath.F64Vec3 point)
        {
            return _centerline.MapPointToPathDistance(point);
        }

        public struct Triangle
        {
            public readonly FixMath.F64Vec3 A;
            public readonly FixMath.F64Vec3 Edge0;
            public readonly FixMath.F64Vec3 Edge1;

            internal FixMath.F64Vec3 Tangent;
            internal FixMath.F64Vec3 PointOnPath;

            internal readonly FixMath.F64 Determinant;

            public Triangle(FixMath.F64Vec3 a, FixMath.F64Vec3 b, FixMath.F64Vec3 c)
            {
                A = a;
                Edge0 = b - a;
                Edge1 = c - a;

                PointOnPath = FixMath.F64Vec3.Zero;
                Tangent = FixMath.F64Vec3.Zero;

                // ReSharper disable once ImpureMethodCallOnReadonlyValueField
                var edge0LengthSquared = FixMath.F64Vec3.LengthSqr(Edge0);

                var edge0DotEdge1 = FixMath.F64Vec3.Dot(Edge0, Edge1);
                var edge1LengthSquared = FixMath.F64Vec3.Dot(Edge1, Edge1);

                Determinant = edge0LengthSquared * edge1LengthSquared - edge0DotEdge1 * edge0DotEdge1;
            }
        }

        private static FixMath.F64Vec3 ClosestPointOnTriangle(ref Triangle triangle, FixMath.F64Vec3 sourcePosition, out bool inside)
        {
            FixMath.F64 a, b;
            return ClosestPointOnTriangle(ref triangle, sourcePosition, out a, out b, out inside);
        }

        internal static FixMath.F64Vec3 ClosestPointOnTriangle(ref Triangle triangle, FixMath.F64Vec3 sourcePosition, out FixMath.F64 edge0Distance, out FixMath.F64 edge1Distance, out bool inside)
        {
            FixMath.F64Vec3 v0 = triangle.A - sourcePosition;

            // ReSharper disable once ImpureMethodCallOnReadonlyValueField
            var a = FixMath.F64Vec3.LengthSqr(triangle.Edge0);
            var b = FixMath.F64Vec3.Dot(triangle.Edge0, triangle.Edge1);
            // ReSharper disable once ImpureMethodCallOnReadonlyValueField
            var c = FixMath.F64Vec3.LengthSqr(triangle.Edge1);
            var d = FixMath.F64Vec3.Dot(triangle.Edge0, v0);
            var e = FixMath.F64Vec3.Dot(triangle.Edge1, v0);

            var det = triangle.Determinant;
            var s = b * e - c * d;
            var t = b * d - a * e;

            inside = false;
            if (s + t < det)
            {
                if (s < 0)
                {
                    if (t < 0)
                    {
                        if (d < 0)
                        {
                            s = Utilities.Clamp(-d / a, FixMath.F64.Zero, FixMath.F64.One);
                            t = FixMath.F64.Zero;
                        }
                        else
                        {
                            s = FixMath.F64.Zero;
                            t = Utilities.Clamp(-e / c, FixMath.F64.Zero, FixMath.F64.One);
                        }
                    }
                    else
                    {
                        s = FixMath.F64.Zero;
                        t = Utilities.Clamp(-e / c, FixMath.F64.Zero, FixMath.F64.One);
                    }
                }
                else if (t < 0)
                {
                    s = Utilities.Clamp(-d / a, FixMath.F64.Zero, FixMath.F64.One);
                    t = FixMath.F64.Zero;
                }
                else
                {
                    var invDet = 1 / det;
                    s *= invDet;
                    t *= invDet;
                    inside = true;
                }
            }
            else
            {
                if (s < 0)
                {
                    var tmp0 = b + d;
                    var tmp1 = c + e;
                    if (tmp1 > tmp0)
                    {
                        var numer = tmp1 - tmp0;
                        var denom = a - 2 * b + c;
                        s = Utilities.Clamp(numer / denom, FixMath.F64.Zero, FixMath.F64.One);
                        t = 1 - s;
                    }
                    else
                    {
                        t = Utilities.Clamp(-e / c, FixMath.F64.Zero, FixMath.F64.One);
                        s = FixMath.F64.Zero;
                    }
                }
                else if (t < 0)
                {
                    if (a + d > b + e)
                    {
                        var numer = c + e - b - d;
                        var denom = a - 2 * b + c;
                        s = Utilities.Clamp(numer / denom, FixMath.F64.Zero, FixMath.F64.One);
                        t = 1 - s;
                    }
                    else
                    {
                        s = Utilities.Clamp(-e / c, FixMath.F64.Zero, FixMath.F64.One);
                        t = FixMath.F64.Zero;
                    }
                }
                else
                {
                    var numer = c + e - b - d;
                    var denom = a - 2 * b + c;
                    s = Utilities.Clamp(numer / denom, FixMath.F64.Zero, FixMath.F64.One);
                    t = 1 - s;
                }
            }

            edge0Distance = s;
            edge1Distance = t;
            return triangle.A + s * triangle.Edge0 + t * triangle.Edge1;
        }
    }
}
