using System;
using System.Collections.Generic;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Recast.Toolset.Builder;
using DotRecast.Recast.Demo.Draw;
using DotRecast.Recast.Toolset;
using DotRecast.Recast.Toolset.Tools;
using ImGuiNET;
using Serilog;
using static DotRecast.Recast.Demo.Draw.DebugDraw;
using static DotRecast.Recast.Demo.Draw.DebugDrawPrimitives;
using Silk.NET.GLFW;
using Newtonsoft.Json.Linq;
using hxDaedalus.factories;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.IO;
using UnityEngine;
using DotRecast.Core.Collections;
using haxe.lang;
using DotRecast.Pathfinding.Crowds;
using DotRecast.Pathfinding.Util;
using hxDaedalus.ai;
using Serilog.Core;
using Silk.NET.Windowing;
using SharpSteer2.Helpers;

namespace DotRecast.Recast.Demo.Tools;

public class TestDaedalusToolMode
{
    public static readonly TestDaedalusToolMode ADD_OBSTACLE = new TestDaedalusToolMode(0, "Add Obstacle");
    public static readonly TestDaedalusToolMode PATH_FINDER = new TestDaedalusToolMode(1, "Path Finder");
    public static readonly TestDaedalusToolMode ADD_CROWD_ENTITY = new TestDaedalusToolMode(2, "Add Crowd Entity");
    public static readonly TestDaedalusToolMode MOVE_CROWD_ENTITY = new TestDaedalusToolMode(3, "Move Crowd Entity");

    public static readonly RcImmutableArray<TestDaedalusToolMode> Values = RcImmutableArray.Create(
            ADD_OBSTACLE,
            PATH_FINDER,
            ADD_CROWD_ENTITY,
            MOVE_CROWD_ENTITY
        );

    public readonly int Idx;
    public readonly string Label;

    private TestDaedalusToolMode(int idx, string label)
    {
        Idx = idx;
        Label = label;
    }
}

public class TestDaedalusTool : IRcToolable, IPathwayQuerier, ILocalBoundaryQuerier
{
    // pseudo random generator
    Random rand = new Random((int)DateTime.Now.Ticks);

    // Graph mesh
    public hxDaedalus.data.Mesh Mesh { get; private set; }

    public UnityEngine.Vector3? StartPoint { get; set; }
    public UnityEngine.Vector3? EndPoint { get; set; }

    public hxDaedalus.data.Face HitFace { get; set; }

    // map height
    public float MapHeight { get; set; }

    // pathfinder
    public hxDaedalus.ai.PathFinder Pathfinder { get; private set; }
    public hxDaedalus.ai.EntityAI EntityAI { get; private set; }
    public HxArray<double> Path { get; private set; }

    // obstacles
    private HxArray<hxDaedalus.data.Object> _obstacles = new HxArray<hxDaedalus.data.Object>();

    // crowd entities
    private IMovableEntityManager _entityManager = new MovableEntityManager();

    public float EntityRadius 
    { 
        get {
            return (float)EntityAI.get_radius();
        } 

        set {
            EntityAI.set_radius(value);
        }
    }

    double RandomRange(double min, double max)
    {
        return min + rand.NextDouble() * (max - min);
    }

    public string GetName()
    {
        return "Test Daedalus Tool";
    }

    public void Update(double inDeltaTime)
    {
        if (null != _entityManager)
        {
            _entityManager.Tick(FixMath.F64.FromDouble(inDeltaTime));
        }
    }

    public void AddObstacle(double x, double y)
    {
        if (null == Mesh) return;
        var hxObject = new hxDaedalus.data.Object();
        var shapeCoords = new HxArray<double>(new double[] {
                            -1, -1, 1, -1,
                             1, -1, 1, 1,
                             1, 1, -1, 1,
                            -1, 1, -1, -1});

        hxObject._coordinates = shapeCoords;
        // randGen.rangeMin = 10;
        // randGen.rangeMax = 40;
        hxObject._scaleX = RandomRange(10, 40) / 600.0f * Mesh._width;
        hxObject._scaleY = RandomRange(10, 40) / 600.0f * Mesh._height;
        // randGen.rangeMin = 0;
        // randGen.rangeMax = 1000;
        hxObject._rotation = (RandomRange(0, 1000) / 1000) * Math.PI / 2;
        // randGen.rangeMin = 50;
        // randGen.rangeMax = 600;
        hxObject._x = x;
        hxObject._y = y;
        Mesh.insertObject(hxObject);
        _obstacles.push(hxObject);
    }

    public void RemoveObstacle(hxDaedalus.data.Object obstacle)
    {
        if (null == Mesh) return;
        Mesh.deleteObject(obstacle);
        _obstacles.remove(obstacle);
    }

    public hxDaedalus.data.Object HitObstacle(double x, double y)
    {
        var p = new hxDaedalus.data.math.Point2D(x, y);

        for (var i = 0; i < _obstacles.length; ++i)
        {
            // var m = _obstacles[i].get_matrix();    
            var o = _obstacles[i];
            var edges = _obstacles[i].get_edges();
            if (edges.length == 0) continue;

            // 
            var matrix = new hxDaedalus.data.math.Matrix2D(haxe.lang.EmptyObject.EMPTY);
            matrix.identity();
            matrix.translate(-o._x, -o._y);
            matrix.rotate(-o.get_rotation());

            var local_p = p.clone();
            matrix.tranform(local_p);

            if (Math.Abs(local_p.x) < o._scaleX && Math.Abs(local_p.y) < o._scaleY)
            {
                return o;
            }
            
            //var containt = true;
            //var last = 0.0;
            //for (var j=0; j<edges.length; ++j)
            //{
            //    var edge = edges[j] as hxDaedalus.data.Edge;
            //    var v0 = edge.get_originVertex().get_pos();
            //    var v1 = edge.get_destinationVertex().get_pos();
            //    var dir = v1; dir.substract(v0);

            //    var v = p; v.substract(v0);
            //    // cross(v, dir)
            //    var cross = v.x * dir.y - v.y * dir.x;
            //    if (j == 0)
            //    {
            //        last = cross;
            //    }
            //    else if (last * cross < 0)
            //    {
            //        containt = false;
            //        break;
            //    }
            //}

            //if (containt) 
            //{
            //    return _obstacles[i];
            //}
        }
        return null;
    }

    public bool BuildGraphMesh(double x, double y, double mapWidth, double mapHeight)
    {
        // build a rectangular 2 polygons mesh of mapWidth x mapHeight
        var mesh = RectMesh.buildRectangle(mapWidth, mapHeight);
        for (var i = 0; i < mesh._vertices.length; ++i)
        { 
            var vertex = mesh._vertices[i] as hxDaedalus.data.Vertex;
            vertex._pos.x += x;
            vertex._pos.y += y;
        }

        // populate mesh with many square objects
        hxDaedalus.data.Object hxObject = null;
        HxArray<double> shapeCoords = null;
        for (int i=0; i<30; ++i)
        {
            hxObject = new hxDaedalus.data.Object();
            shapeCoords = new HxArray<double>(new double[] {
                            -1, -1, 1, -1,
                             1, -1, 1, 1,
                             1, 1, -1, 1,
                            -1, 1, -1, -1});

            hxObject._coordinates = shapeCoords;
            // randGen.rangeMin = 10;
            // randGen.rangeMax = 40;
            hxObject._scaleX = RandomRange(10, 40) / 600.0f * mapWidth;
            hxObject._scaleY = RandomRange(10, 40) / 600.0f * mapHeight;
            // randGen.rangeMin = 0;
            // randGen.rangeMax = 1000;
            hxObject._rotation = (RandomRange(0, 1000) / 1000) * Math.PI / 2;
            // randGen.rangeMin = 50;
            // randGen.rangeMax = 600;
            hxObject._x = x + RandomRange(50, 600) / 600.0f * mapWidth;
            hxObject._y = y + RandomRange(50, 600) / 600.0f * mapHeight;
            mesh.insertObject(hxObject);
            _obstacles.push(hxObject);
        }  // show result mesh on screen

        Mesh = mesh;

        // we need an entity
        EntityAI = new hxDaedalus.ai.EntityAI();
        // set radius as size for your entity
        EntityAI.set_radius(0.5);
        // set a position
        EntityAI.x = 20;
        EntityAI.y = 20;

        // pathfinder
        Pathfinder = new hxDaedalus.ai.PathFinder();
        Pathfinder.entity = EntityAI;
        Pathfinder.set_mesh(mesh);

        Path = new HxArray<double>();

        return true;
    }

    // IPathwayQuerier interface
    public virtual SharpSteer2.Pathway.PolylinePathway FindPath(SharpSteer2.IVehicle vehicle, FixMath.F64Vec3 target)
    {
        var pathfinder = Pathfinder;
        if (null == pathfinder) return null;

        var start = vehicle.PredictFuturePosition(FixMath.F64.Zero);
        // 1.Create template entity
        var entityAI = new hxDaedalus.ai.EntityAI();
        EntityAI.set_radius(vehicle.Radius.Double);
        EntityAI.x = start.X.Double;
        EntityAI.y = start.Z.Double;

        // 2.Set pathfinder params
        pathfinder.entity = EntityAI;

        var resultPath = new HxArray<double>();
        pathfinder.findPath(target.X.Double, target.Z.Double, resultPath);

        // 3.Convert to PolylinePathway 
        if (resultPath.length > 0)
        {
            var points = new List<FixMath.F64Vec3>();
            for (var i = 0; i < resultPath.length; i += 2)
            {
                var v = FixMath.F64Vec3.FromFloat((float)resultPath[0], MapHeight, (float)resultPath[1]);
                points.Add(v);
            }
            return new SharpSteer2.Pathway.PolylinePathway(points, FixMath.F64.Zero, false);
        }
        return null;
    }
    // End

    // ILocalBoundaryQuerier interface
    public virtual int QueryBoundaryInCircle(SharpSteer2.IVehicle vehicle, FixMath.F64 inRadius, BoundarySegement[] outResults)
    {
        if (null == outResults) return 0;
        var center = vehicle.PredictFuturePosition(FixMath.F64.Zero);
        hxDaedalus.data.math.Intersection loc = hxDaedalus.data.math.Geom2D.locatePosition(center.X.Double, center.Z.Double, Mesh);
        hxDaedalus.data.Face refFace = null;
        switch (loc._hx_index)
        {
            case 0:
                {
                    break;
                }
            case 1:
                {
                    break;
                }
            case 2:
                {
                    global::hxDaedalus.data.Face face = (loc as global::hxDaedalus.data.math.Intersection_EFace).face;
                    refFace = face;
                    break;
                }
        }

        var radiusSqured = inRadius * inRadius;

        System.Func<hxDaedalus.data.Vertex, FixMath.F64Vec3> VertextToF64Vec3 = (vertex) => {
            var pos = vertex.get_pos();
            return FixMath.F64Vec3.FromDouble(pos.x, MapHeight, pos.y);
        };

        System.Func<hxDaedalus.data.Face, bool> CheckVertexInRadius = (face) => {
            global::hxDaedalus.iterators.FromFaceToInnerEdges iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
            iterEdge.set_fromFace(face);
            while (true)
            {
                var innerEdge = iterEdge.next();
                if (innerEdge == null)
                {
                    break;
                }

                var pos = VertextToF64Vec3(innerEdge.get_originVertex());
                var distSq = center.Distance2D(pos);
                if (distSq <= radiusSqured) return true;
            }
            return false;
        };

        if (null != refFace)
        {
            // Boundary edges
            var boundaryEdges = new List<hxDaedalus.data.Edge>();
            // Visit adjacent triangles
            var pendingVisitFaces = new Queue<hxDaedalus.data.Face>();
            var visiedFaces = new HashSet<hxDaedalus.data.Face>();

            pendingVisitFaces.Enqueue(refFace);
            var innerEdges = new hxDaedalus.iterators.FromFaceToInnerEdges();
            while (pendingVisitFaces.Count > 0)
            {
                var face = pendingVisitFaces.Dequeue();
                visiedFaces.Add(face);
                // 遍历相邻的face
                innerEdges.set_fromFace(face);
                while (true)
                {
                    var innerEdge = innerEdges.next();
                    if (innerEdge == null) break;
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
                        if (visiedFaces.Contains(face))
                            continue;
                        // 检查是否在范围内
                        if (!CheckVertexInRadius(adjacentFace))
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
                    outResults[i].Start = VertextToF64Vec3(boundaryEdges[i].get_originVertex());
                    outResults[i].End = VertextToF64Vec3(boundaryEdges[i].get_destinationVertex());
                }
                return count;
            }
        }

        return 0;
    }
    // End

    // Crowd entity interface
    public UniqueId AddCrowdEntity(double x, double y, double r)
    {
        var Params = new CreateEntityParams() 
        {
            EntityId = UniqueId.InvalidID,
            SpawnPosition = FixMath.F64Vec3.Zero,
            SpawnRotation = FixMath.F64Quat.Identity,

            PathwayQuerier = null,
            LocalBoundaryQuerier = null,
            AnnotationService = null,
        };

        return _entityManager.CreateEntity(Params);
    }

    public void RemoveCrowdEntity(UniqueId InEntityId)
    {
        _entityManager.DeleteEntity(InEntityId);
    }

    public UniqueId HitCrowdEntity(double x, double y)
    {
        return UniqueId.InvalidID;
    }

    public void MoveCrowdEntity(double x, double y)
    {
        var Position = FixMath.F64Vec3.FromDouble(x, MapHeight, y);
        _entityManager.ForEachEntity((InEntity) => { 
            if (null != InEntity)
            {
                InEntity.TargetLocation = Position;
            }
        });
    }
    // End
}

public class TestDaedalusSampleTool : ISampleTool
{
    private static readonly ILogger Logger = Log.ForContext<TestNavmeshSampleTool>();

    private DemoSample _sample;

    private TestDaedalusTool _tool = new TestDaedalusTool();

    private TileDelaunayDebugDraw _draw = null;

    private hxDaedalus.view.SimpleView _view = null;

    private TestDaedalusToolMode m_mode = TestDaedalusToolMode.ADD_OBSTACLE;
    private int m_modeIdx = TestDaedalusToolMode.ADD_OBSTACLE.Idx;

    public IRcToolable GetTool()
    {
        return _tool;
    }

    public void HandleClick(RcVec3f s, RcVec3f p, bool shift)
    {
    }

    public void HandleClickRay(RcVec3f start, RcVec3f direction, bool shift)
    {
        if (null == _draw) return;
        if (null == _tool.Mesh) return;

        // 平面 y = _draw.MapHeight

        var src = start;
        var dst = start + direction * 100.0f;
        var bmin = new RcVec3f(0.0f, _draw.MapHeight, 0.0f);
        var bmax = new RcVec3f((float)_tool.Mesh._width, _draw.MapHeight, (float)_tool.Mesh._height);
        if (!RcIntersections.IsectSegAABB(src, dst, bmin, bmax, out var btmin, out var btmax))
        {
            return;
        }

        var hitPos = src + (dst - src) * btmin;

        Logger.Information($"HandleClickRay, hitPos:{hitPos}");

        hxDaedalus.data.math.Intersection loc = hxDaedalus.data.math.Geom2D.locatePosition(hitPos.X, hitPos.Z, _tool.Mesh);
        switch (loc._hx_index)
        {
            case 0:
                {
                    //global::hxDaedalus.data.Vertex vertex = (loc as global::hxDaedalus.data.math.Intersection_EVertex).vertex;
                    //locVertex = vertex;
                    //return;
                    break;
                }
            case 1:
                {
                    //global::hxDaedalus.data.Edge edge = (loc as global::hxDaedalus.data.math.Intersection_EEdge).edge;
                    //{
                    //    locEdge = edge;
                    //    if (locEdge.get_isConstrained())
                    //    {
                    //        return;
                    //    }

                    //    this.fromFace = locEdge.get_leftFace();
                    //}

                    break;
                }
            case 2:
                {
                    global::hxDaedalus.data.Face face = (loc as global::hxDaedalus.data.math.Intersection_EFace).face;
                    Logger.Information($"HandleClickRay, hit face:{face._id}");
                    _tool.HitFace = face;
                    break;
                }
            case 3:
                {
                    break;
                }
        }

        if (m_mode == TestDaedalusToolMode.ADD_OBSTACLE)
        {
            if (shift)
            {
                // Delete
                var obstacle = _tool.HitObstacle(hitPos.X, hitPos.Z);
                if (null != obstacle) _tool.RemoveObstacle(obstacle);
            }
            else
            {
                // Add
                _tool.AddObstacle(hitPos.X, hitPos.Z);
            }
        }
        else if (m_mode == TestDaedalusToolMode.PATH_FINDER)
        {
            if (_tool.StartPoint == null)
            {
                _tool.StartPoint = new UnityEngine.Vector3(hitPos.X, hitPos.Y, hitPos.Z);
            }
            else
            {
                if (_tool.EndPoint != null)
                    _tool.StartPoint = _tool.EndPoint;
                _tool.EndPoint = new UnityEngine.Vector3(hitPos.X, hitPos.Y, hitPos.Z);

                _tool.EntityAI.x = _tool.StartPoint.Value.x;
                _tool.EntityAI.y = _tool.StartPoint.Value.z;
                _tool.EntityAI._x = _tool.EntityAI.x;
                _tool.EntityAI._y = _tool.EntityAI.y;

                _tool.Pathfinder.findPath(_tool.EndPoint.Value.x, _tool.EndPoint.Value.z, _tool.Path);

                Logger.Information($"HandleClickRay, findPath length:{_tool.Path.length}");
            }
        }
        else if (m_mode == TestDaedalusToolMode.ADD_CROWD_ENTITY)
        {
            var radius = 0.5f;
            if (shift)
            {
                // Delete
                var entityId = _tool.HitCrowdEntity(hitPos.X, hitPos.Z);
                if (entityId != UniqueId.InvalidID)
                    _tool.RemoveCrowdEntity(entityId);
            }
            else
            {
                // Add
                _tool.AddCrowdEntity(hitPos.X, hitPos.Z, radius);
            }
        }
        else if (m_mode == TestDaedalusToolMode.MOVE_CROWD_ENTITY)
        { 
            _tool.MoveCrowdEntity(hitPos.X, hitPos.Z);
        }
    }

    public void HandleRender(NavMeshRenderer renderer)
    {
        var dd = renderer.GetDebugDraw();

        var geom = _sample.GetInputGeom();
        var bound_min = geom.GetMeshBoundsMin();
        var bound_max = geom.GetMeshBoundsMax();

        if (_draw == null)
        {
            _draw = new TileDelaunayDebugDraw(dd);
            _draw.MapHeight = bound_max.Y + 0.5f;

            _view = new hxDaedalus.view.SimpleView(_draw);
        }

        // draw bounds
        dd.DebugDrawBoxWire(
            bound_min.X, bound_min.Y - 3.0f, bound_min.Z,
            bound_max.X, bound_max.Y + 3.0f, bound_max.Z,
            DebugDraw.DuRGBA(255, 255, 255, 128), 1.0f);

        // draw graph mesh
        if (null != _tool.Mesh) _view.drawMesh(_tool.Mesh);

        if (m_mode == TestDaedalusToolMode.PATH_FINDER)
        {
            var CubeSize = new UnityEngine.Vector3(1.0f, 2.0f, 1.0f);
            if (null != _tool.StartPoint)
                _draw.DrawCube(_tool.StartPoint.Value, CubeSize, UnityEngine.Color.gray);
            if (null != _tool.EndPoint)
                _draw.DrawCube(_tool.EndPoint.Value, CubeSize, UnityEngine.Color.yellow);

            // draw path
            if (_tool.Path.length > 0)
            {
                var v0 = new UnityEngine.Vector3((float)_tool.Path[0], _draw.MapHeight, (float)_tool.Path[1]);
                var PointSize = UnityEngine.Vector3.one * 0.2f;
                _draw.DrawCube(v0, PointSize, UnityEngine.Color.red);
                for (var i = 2; i < _tool.Path.length; i += 2)
                {
                    var v1 = new UnityEngine.Vector3((float)_tool.Path[i], _draw.MapHeight, (float)_tool.Path[i + 1]);
                    _draw.DrawLine(v0, v1, UnityEngine.Color.green);
                    _draw.DrawCube(v1, PointSize, UnityEngine.Color.red);
                    v0 = v1;
                }
            }
        }

        // draw face
        for (var i=0; i<_tool.Mesh._faces.length; ++i)
        {
            var face = (hxDaedalus.data.Face)_tool.Mesh._faces[i];
            if ( face == _tool.HitFace)
            {
                var h = _draw.MapHeight;
                _draw.MapHeight = h + 0.5f;

                global::hxDaedalus.iterators.FromFaceToInnerEdges iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
                iterEdge.set_fromFace(face);
                while (true) {
					var innerEdge = iterEdge.next();
					if ( innerEdge == null ) 
                    {
						break;
					}

                    _view.drawEdge(innerEdge, UnityEngine.Color.magenta, 10.0f);
                }

                _draw.MapHeight = h;

                var color = UnityEngine.Color.yellow;
                color.a *= 0.1f;
                _view.drawFace(face, color);
                break;
            }
        }
    }

    public void HandleUpdate(float dt)
    {
        if (null != _tool)
        {
            _tool.Update(dt);
        }
    }

    public void Layout()
    {
        ImGui.Text($"Test Daedalus Tool Mode");
        ImGui.Separator();

        TestDaedalusToolMode previousToolMode = m_mode;
        ImGui.RadioButton(TestDaedalusToolMode.ADD_OBSTACLE.Label, ref m_modeIdx, TestDaedalusToolMode.ADD_OBSTACLE.Idx);
        ImGui.RadioButton(TestDaedalusToolMode.PATH_FINDER.Label, ref m_modeIdx, TestDaedalusToolMode.PATH_FINDER.Idx);
        ImGui.RadioButton(TestDaedalusToolMode.ADD_CROWD_ENTITY.Label, ref m_modeIdx, TestDaedalusToolMode.ADD_CROWD_ENTITY.Idx);
        ImGui.RadioButton(TestDaedalusToolMode.MOVE_CROWD_ENTITY.Label, ref m_modeIdx, TestDaedalusToolMode.MOVE_CROWD_ENTITY.Idx);
        ImGui.NewLine();

        if (previousToolMode.Idx != m_modeIdx)
        {
            m_mode = TestDaedalusToolMode.Values[m_modeIdx];
        }

        var EntityRadius = _tool.EntityRadius;
        ImGui.SliderFloat("Entity Radius", ref EntityRadius, 0.1f, 3.0f, "%.2f");
        _tool.EntityRadius = EntityRadius;
    }

    public void OnSampleChanged()
    {
    }

    public void SetSample(DemoSample sample)
    {
        _sample = sample;

        var geom = _sample.GetInputGeom();
        var bound_min = geom.GetMeshBoundsMin();
        var bound_max = geom.GetMeshBoundsMax();

        var bound_center = (bound_min + bound_max) * 0.5f;

        var mapWidth = bound_max.X - bound_min.X;
        var mapHeight = bound_max.Z - bound_min.Z;

        _tool.BuildGraphMesh(0, 0, mapWidth, mapHeight);

        Logger.Information($"init graph mesh, mapWidth:{mapWidth}, mapHeight:{mapHeight}");
    }
}
