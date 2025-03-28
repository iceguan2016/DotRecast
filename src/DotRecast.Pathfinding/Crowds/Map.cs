
using System;
using System.Collections.Generic;
using FixMath;
using Pathfinding.Triangulation.AI;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Factories;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Triangulation.Math;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Pathway;
using Object = Pathfinding.Triangulation.Data.Object;

namespace Pathfinding.Crowds
{
    public class Map : ILocalBoundaryQuerier, IPathwayQuerier
    {
        private Mesh _navmesh = null;

        // map height
        public FixMath.F64 MapHeight { get; set; }

        // pathfinder
        private PathFinder _pathfinder = null;
        public PathFinder Pathfinder { get { return _pathfinder; } }

        // obstacles
        private List<Object> _obstacles = new List<Object>();

        // Map默认中心在(0,0,0)坐标,所以只需要传入width和height
        public bool SetMapInfo(FixMath.F64 mapWidth, FixMath.F64 mapHeight)
        {
            // build a rectangular 2 polygons mesh of mapWidth x mapHeight
            var mesh = RectMesh.buildRectangle(mapWidth, mapHeight);
            if (null == mesh)
            {
                return false;
            }

            _navmesh = mesh;
            _pathfinder = new PathFinder();
            _pathfinder.set_mesh(mesh);
            return true;
        }

        public Object AddObstacle(FixMath.F64Vec3 pos, FixMath.F64Quat rot, FixMath.F64Vec3 extent)
        {
            if (null == _navmesh)
                return null;
            var hxObject = new Object();
            var shapeCoords = new List<FixMath.F64> {
                            -FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One,
                             FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One, FixMath.F64.One,
                             FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One,
                            -FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One, -FixMath.F64.One };

            hxObject._coordinates = shapeCoords;
            hxObject._scaleX = extent.X;
            hxObject._scaleY = extent.Z;
            if (FixMath.F64Quat.ToEulerAnglesRad(rot, out var eulerAngle))
            {
                hxObject._rotation = eulerAngle.Y;
            }
            else
            {
                hxObject._rotation = FixMath.F64.Zero;
            }
            hxObject._x = pos.X;
            hxObject._y = pos.Z;

            _navmesh.insertObject(hxObject);
            _obstacles.Add(hxObject);
            return hxObject;
        }

        public void RemoveObstacle(Object obstacle)
        {
            if (null == _navmesh)
                return;
            _navmesh.deleteObject(obstacle);
            _obstacles.Remove(obstacle);
        }

        #region ILocalBoundaryQuerier
        public int QueryBoundaryInCircle(IVehicle vehicle, F64 inRadius, BoundarySegement[] outResults)
        {
            if (null == outResults)
                return 0;
            var center = vehicle.PredictFuturePosition(FixMath.F64.Zero);
            var loc = Geom2D.locatePosition(center.X, center.Z, _navmesh);
            Face refFace = null;
            if (loc is Intersection_EVertex v)
            {
            }
            else if (loc is Intersection_EEdge e)
            {

            }
            else if (loc is Intersection_EFace f)
            {
                refFace = f.face;
            }

            var radiusSqured = inRadius * inRadius;

            System.Func<hxDaedalus.data.Vertex, FixMath.F64Vec3> VertextToF64Vec3 = (vertex) =>
            {
                var pos = vertex.get_pos();
                return FixMath.F64Vec3.FromDouble(pos.x, MapHeight.Double, pos.y);
            };

            System.Func<Face, bool> CheckIntersectionFaceAndCircle = (face) =>
            {
                // 这里改成Face的AABB盒是否和Circle相交判定，比只判定3个顶点更准确一些
                FromFaceToInnerEdges iterEdge = new FromFaceToInnerEdges();
                iterEdge.set_fromFace(face);
                var min = new FixMath.F64Vec3(FixMath.F64.MaxValue, FixMath.F64.MaxValue, FixMath.F64.MaxValue);
                var max = new FixMath.F64Vec3(FixMath.F64.MinValue, FixMath.F64.MinValue, FixMath.F64.MinValue);
                while (true)
                {
                    var innerEdge = iterEdge.next();
                    if (innerEdge == null)
                    {
                        break;
                    }

                    var pos = innerEdge.get_originVertex();
                    min.X = FixMath.F64.Min(min.X, pos._pos.X);
                    min.Z = FixMath.F64.Min(min.Z, pos._pos.Y);

                    max.X = FixMath.F64.Max(max.X, pos._pos.X);
                    max.Z = FixMath.F64.Max(max.Z, pos._pos.Y);
                }

                if (min.X <= max.X && min.Z <= max.Z)
                {
                    var radius_v3 = new FixMath.F64Vec3(inRadius, inRadius, inRadius);
                    var circle_min = center - radius_v3;
                    var circle_max = center + radius_v3;

                    // 逐维度比较，如果有一个维度不相交，则返回 false
                    if (max.X < circle_min.X || min.X > circle_max.X)
                    {
                        return false;
                    }
                    //if (max.Y < circle_min.Y || min.Y > circle_max.Y)
                    //{
                    //    return false;
                    //}
                    if (max.Z < circle_min.Z || min.Z > circle_max.Z)
                    {
                        return false;
                    }

                    return true;
                }
                return false;
            };

            if (null != refFace)
            {
                // Boundary edges
                var boundaryEdges = new List<Edge>();
                // Visit adjacent triangles
                var pendingVisitFaces = new Queue<Face>();
                var visiedFaces = new HashSet<Face>();

                pendingVisitFaces.Enqueue(refFace);
                var innerEdges = new FromFaceToInnerEdges();
                while (pendingVisitFaces.Count > 0)
                {
                    var face = pendingVisitFaces.Dequeue();
                    visiedFaces.Add(face);
                    // 遍历相邻的face
                    innerEdges.set_fromFace(face);
                    while (true)
                    {
                        var innerEdge = innerEdges.next();
                        if (innerEdge == null)
                            break;
                        // 判断是否是Boundary
                        if (innerEdge.get_isConstrained())
                        {
                            boundaryEdges.Add(innerEdge);
                        }
                        else
                        {
                            //  获取邻接face
                            var outterEdge = innerEdge.get_oppositeEdge();
                            if (outterEdge == null)
                                continue;
                            var adjacentFace = outterEdge.get_leftFace();
                            if (adjacentFace == null)
                                continue;
                            // 检查是否已经访问过
                            if (visiedFaces.Contains(adjacentFace))
                                continue;
                            // 检查是否在范围内
                            if (!CheckIntersectionFaceAndCircle(adjacentFace))
                                continue;
                            pendingVisitFaces.Enqueue(adjacentFace);
                        }
                    }
                }

                // 输出boundary
                if (boundaryEdges.Count > 0)
                {
                    var count = Math.Min(boundaryEdges.Count, outResults.Length);
                    for (int i = 0; i < count; ++i)
                    {
                        outResults[i].Start = boundaryEdges[i].get_originVertex()._pos.Cast(MapHeight);
                        outResults[i].End = boundaryEdges[i].get_destinationVertex()._pos.Cast(MapHeight);
                    }
                    return count;
                }
            }

            return 0;
        }
        #endregion

        #region IPathwayQuerier
        public PolylinePathway FindPath(IVehicle vehicle, F64Vec3 target)
        {
            var pathfinder = Pathfinder;
            if (null == pathfinder)
                return null;

            var start = vehicle.PredictFuturePosition(FixMath.F64.Zero);
            var radius = vehicle.Radius;

            // 1.Create template entity

            // 2.Set pathfinder params
            var resultPath = new List<FixMath.F64>();
            pathfinder.findPath(start.X, start.Z, target.X, target.Z, radius, resultPath);

            // 3.Convert to PolylinePathway 
            if (resultPath.Count > 0)
            {
                var points = new List<FixMath.F64Vec3>();
                for (var i = 0; i < resultPath.Count; i += 2)
                {
                    var v = new FixMath.F64Vec3(resultPath[i], MapHeight, resultPath[i + 1]);
                    points.Add(v);
                }
                return new SharpSteer2.Pathway.PolylinePathway(points, FixMath.F64.Zero, false);
            }
            return null;
        }
        #endregion
    }
}
