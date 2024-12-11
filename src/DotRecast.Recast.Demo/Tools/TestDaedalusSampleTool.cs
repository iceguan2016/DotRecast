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

namespace DotRecast.Recast.Demo.Tools;

public class TestDaedalusTool : IRcToolable
{
    // pseudo random generator
    Random rand = new Random((int)DateTime.Now.Ticks);

    // Graph mesh
    public hxDaedalus.data.Mesh Mesh { get; private set; }

    public UnityEngine.Vector3? StartPoint { get; set; }
    public UnityEngine.Vector3? EndPoint { get; set; }

    public hxDaedalus.data.Face HitFace { get; set; }

    // pathfinder
    public hxDaedalus.ai.PathFinder Pathfinder { get; private set; }
    public hxDaedalus.ai.EntityAI EntityAI { get; private set; }
    public HxArray<double> Path { get; private set; }

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

    public bool BuildGraphMesh(double mapWidth, double mapHeight)
    {
        // build a rectangular 2 polygons mesh of mapWidth x mapHeight
        var mesh = RectMesh.buildRectangle(mapWidth, mapHeight);

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
            hxObject._x = RandomRange(50, 600) / 600.0f * mapWidth;
            hxObject._y = RandomRange(50, 600) / 600.0f * mapHeight;
            mesh.insertObject(hxObject);
        }  // show result mesh on screen

        Mesh = mesh;

        // we need an entity
        EntityAI = new hxDaedalus.ai.EntityAI();
        // set radius as size for your entity
        EntityAI.set_radius(0.0);
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
}

public class TestDaedalusSampleTool : ISampleTool
{
    private static readonly ILogger Logger = Log.ForContext<TestNavmeshSampleTool>();

    private DemoSample _sample;

    private TestDaedalusTool _tool = new TestDaedalusTool();

    private TileDelaunayDebugDraw _draw = null;

    private hxDaedalus.view.SimpleView _view = null;

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

        // 只打印hitPos
        if (shift) return;

        if (_tool.StartPoint == null)
        {
            _tool.StartPoint = new UnityEngine.Vector3(hitPos.X, hitPos.Y, hitPos.Z);
        }
        else
        {
            if (_tool.EndPoint != null) _tool.StartPoint = _tool.EndPoint;
            _tool.EndPoint = new UnityEngine.Vector3(hitPos.X, hitPos.Y, hitPos.Z);

            _tool.EntityAI.x = _tool.StartPoint.Value.x;
            _tool.EntityAI.y = _tool.StartPoint.Value.z;
            _tool.EntityAI._x = _tool.EntityAI.x;
            _tool.EntityAI._y = _tool.EntityAI.y;

            _tool.Pathfinder.findPath(_tool.EndPoint.Value.x, _tool.EndPoint.Value.z, _tool.Path);

            Logger.Information($"HandleClickRay, findPath length:{_tool.Path.length}");
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

        var CubeSize = new UnityEngine.Vector3(1.0f, 2.0f, 1.0f);
        if (null != _tool.StartPoint) _draw.DrawCube(_tool.StartPoint.Value, CubeSize, UnityEngine.Color.gray);
        if (null != _tool.EndPoint) _draw.DrawCube(_tool.EndPoint.Value, CubeSize, UnityEngine.Color.yellow);

        // draw path
        if (_tool.Path.length > 0)
        {
            var v0 = new UnityEngine.Vector3((float)_tool.Path[0], _draw.MapHeight, (float)_tool.Path[1]);
            var PointSize = UnityEngine.Vector3.one * 0.2f;
            _draw.DrawCube(v0, PointSize, UnityEngine.Color.red);
            for (var i = 2; i < _tool.Path.length; i += 2)
            {
                var v1 = new UnityEngine.Vector3((float)_tool.Path[i], _draw.MapHeight, (float)_tool.Path[i+1]);
                _draw.DrawLine(v0, v1, UnityEngine.Color.green);
                _draw.DrawCube(v1, PointSize, UnityEngine.Color.red);
                v0 = v1;
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

                _view.drawFace(face, UnityEngine.Color.yellow);
                break;
            }
        }
    }

    public void HandleUpdate(float dt)
    {
    }

    public void Layout()
    {
        ImGui.Text($"Test Daedalus Tool Mode");
        ImGui.Separator();

        var EntityRadius = _tool.EntityRadius;
        ImGui.SliderFloat("Obstacle Size", ref EntityRadius, 0.1f, 3.0f, "%.2f");
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

        var mapWidth = bound_max.X - bound_min.X;
        var mapHeight = bound_max.Z - bound_min.Z;

        _tool.BuildGraphMesh(mapWidth, mapHeight);

        Logger.Information($"init graph mesh, mapWidth:{mapWidth}, mapHeight:{mapHeight}");
    }
}
