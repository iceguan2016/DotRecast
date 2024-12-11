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
using Navmesh;
using DotRecast.Core.Collections;
using System.Collections.Generic;
using Navmesh.Shape;
using System;
using DotRecast.Core;
using UnityEngine;
using Game.Utils;
using System.Drawing;
using Silk.NET.SDL;

namespace DotRecast.Recast.Demo.Tools;

public class TileDelaunayNavmeshToolMode
{
    public static readonly TileDelaunayNavmeshToolMode ADD_OBSTACLE = new TileDelaunayNavmeshToolMode(0, "Add Obstacle");
    public static readonly TileDelaunayNavmeshToolMode BUILD = new TileDelaunayNavmeshToolMode(1, "Build");

    public static readonly RcImmutableArray<TileDelaunayNavmeshToolMode> Values = RcImmutableArray.Create(
            ADD_OBSTACLE,
            BUILD
        );

    public readonly int Idx;
    public readonly string Label;

    private TileDelaunayNavmeshToolMode(int idx, string label)
    {
        Idx = idx;
        Label = label;
    }
}

public class TileDelaunayTool : IRcToolable
{
    private List<BoxShape> m_shapes = new List<BoxShape>();
    private RcRand m_rand = new RcRand(DateTime.Now.Ticks);

    private FTiledNavmeshBuilder m_builder;

    public FDebugParams m_debugParams = new FDebugParams();

    public float m_obstacleSize = 5.0f;
    public float m_obstacleSizeVariance = 3.0f;

    public float  m_cellSize = 1.0f;
    public int    m_tileSize = 60;

    public string GetName()
    {
        return "Tile Delaunay Navmesh";
    }

    public BoxShape addShape(RcVec3f p)
    {
        var pos = new Vector3(p.X, p.Y, p.Z);
        var angle = m_rand.Next() * 360 * Mathf.Deg2Rad;
        var rot = Quaternion.AxisAngle(Vector3.up, angle);
        var extent = new Vector3(
            m_obstacleSize + m_rand.Next() * m_obstacleSizeVariance,
            m_obstacleSize + m_rand.Next() * m_obstacleSizeVariance,
            m_obstacleSize + m_rand.Next() * m_obstacleSizeVariance
         );

        var shape = new BoxShape() { 
            Position = pos,
            Rotation = rot,
            HalfExtent = extent * 0.5f,
        };
        m_shapes.Add(shape);
        return shape;
    }

    public void removeShape(BoxShape shape)
    { 
        m_shapes.Remove(shape);
    }

    public BoxShape hitTestShape(RcVec3f s, RcVec3f p)
    { 
        var pos = new Vector3(p.X, p.Y, p.Z);
        for (int i = 0; i < m_shapes.Count; ++i)
        {
            var shape = m_shapes[i];
            if (shape.IsContainPoint(pos))
            {
                return shape;
            }
        }
        return null;
    }

    public List<BoxShape> getAllShapes()
    {
        return m_shapes;
    }

    public FTiledNavmeshBuilder getBuilder()
    {
        return m_builder;
    }

    public FTiledNavmeshGraph buildNavGraph(Vector3 bmin, Vector3 bmax)
    {
        m_builder = new FTiledNavmeshBuilder();

        m_debugParams.IsTrangulation = false;

        // 
        FTiledNavmeshBuilder.FTiledNavmeshBuilderParams builderParams = new FTiledNavmeshBuilder.FTiledNavmeshBuilderParams();
        builderParams.MinBounds = bmin;
        builderParams.MaxBounds = bmax;
        builderParams.CellSize = m_cellSize;
        builderParams.TileSize = m_tileSize;

        var obstacles = new List<FObstacle>();
        for (int i = 0; i < m_shapes.Count; ++i) 
        {
            obstacles.Add(new FObstacle() { Shape = m_shapes[i], IsAdd = true });
        }
        builderParams.Obstacles = obstacles;

        return m_builder.Build(builderParams, m_debugParams);
    }
}

public class TileDelaunayDebugDraw : DrawInterface
{
    private DebugDraw m_draw = null;

    public float MapHeight { get; set; }

    public TileDelaunayDebugDraw(DebugDraw draw)
    {
        m_draw = draw;
        Game.Utils.Debug.drawInterface = this;
    }

    public void DrawCube(Vector3 p, Vector3 size, UnityEngine.Color c)
    {
        if (m_draw != null)
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            var halfSize = size * 0.5f;
            var min = p - halfSize;
            var max = p + halfSize;
            m_draw.DebugDrawBoxWire(min.x, min.y, min.z, max.x, max.y, max.z, color, 1.0f);
        }
    }

    public void DrawCircle(Vector3 p, float r, UnityEngine.Color c)
    {
        if (m_draw != null)
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            m_draw.DebugDrawCircle(p.x, p.y, p.z, r, color, 1.0f);
        }
    }

    public void DrawLine(Vector3 a, Vector3 b, UnityEngine.Color c, float lineWidth = 1.0f)
    {
        if (m_draw != null) 
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            m_draw.Begin(DebugDrawPrimitives.LINES, lineWidth);
            m_draw.Vertex(new float[] { a.x, a.y, a.z }, color);
            m_draw.Vertex(new float[] { b.x, b.y, b.z }, color);
            m_draw.End();
        }
    }

    public void DrawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, UnityEngine.Color c)
    {
        if (m_draw != null)
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            m_draw.Begin(DebugDrawPrimitives.TRIS);
            m_draw.Vertex(new float[] { v0.x, v0.y, v0.z }, color);
            m_draw.Vertex(new float[] { v1.x, v1.y, v1.z }, color);
            m_draw.Vertex(new float[] { v2.x, v2.y, v2.z }, color);
            m_draw.End();
        }
    }

    public float GetMapHeight()
    {
        return MapHeight;
    }
}

public class TileDelaunayNavmeshTool : ISampleTool
{
    private static readonly ILogger Logger = Log.ForContext<TileDelaunayNavmeshTool>();

    private DemoSample _sample;

    private FTiledNavmeshGraph _graph;

    private TileDelaunayTool _tool = new TileDelaunayTool();

    private TileDelaunayDebugDraw _draw = null;

    private TileDelaunayNavmeshToolMode m_mode = TileDelaunayNavmeshToolMode.ADD_OBSTACLE;
    private int m_modeIdx = TileDelaunayNavmeshToolMode.ADD_OBSTACLE.Idx;

    public IRcToolable GetTool()
    {
        return _tool;
    }

    public void HandleClick(RcVec3f s, RcVec3f p, bool shift)
    {
        if (m_mode == TileDelaunayNavmeshToolMode.ADD_OBSTACLE)
        {
            if (shift)
            {
                // Delete
                var hit = _tool.hitTestShape(s, p);
                if (hit != null)
                {
                    _tool.removeShape(hit);
                }
            }
            else
            {
                // Add
                var shape = _tool.addShape(p);
                if (null == shape)
                {
                    Logger.Error("add obstacle failed!");
                }
                else
                {
                    Logger.Information($"add obstacle success, pos:{shape.Position}, rot:{shape.Rotation}, half extent:{shape.HalfExtent}");
                }
            }
        }
    }

    public void HandleClickRay(RcVec3f start, RcVec3f direction, bool shift)
    {
    }

    public void HandleRender(NavMeshRenderer renderer)
    {
        var dd = renderer.GetDebugDraw();

        if (_draw == null)
        {
            _draw = new TileDelaunayDebugDraw(dd);
        }

        var geom = _sample.GetInputGeom();
        var bound_min = geom.GetMeshBoundsMin();
        var bound_max = geom.GetMeshBoundsMax();

        // draw bounds
        dd.DebugDrawBoxWire(
            bound_min.X, bound_min.Y - 3.0f, bound_min.Z, 
            bound_max.X, bound_max.Y + 3.0f, bound_max.Z,
            DebugDraw.DuRGBA(255, 255, 255, 128), 1.0f);

        // draw obstacles
        var obstacles = _tool.getAllShapes();
        for (int i = 0; i < obstacles.Count; i++) 
        {
            var shape = obstacles[i];
            shape.DrawGizmos();
        }

        // draw tile delaunay navmesh
        if (null != _graph)
        {
            _graph.OnDrawGizmos(true);
        }

        // draw builder
        var builder = _tool.getBuilder();
        if (null != builder)
        {
            builder.DrawGizmos(_tool.m_debugParams);
        }
    }

    public void HandleUpdate(float dt)
    {
    }

    public void Layout()
    {
        ImGui.Text($"Tiled Delaunay Navmesh Tool Mode");
        ImGui.Separator();
        TileDelaunayNavmeshToolMode previousToolMode = m_mode;
        ImGui.RadioButton(TileDelaunayNavmeshToolMode.ADD_OBSTACLE.Label, ref m_modeIdx, TileDelaunayNavmeshToolMode.ADD_OBSTACLE.Idx);
        ImGui.RadioButton(TileDelaunayNavmeshToolMode.BUILD.Label, ref m_modeIdx, TileDelaunayNavmeshToolMode.BUILD.Idx);
        ImGui.NewLine();

        if (previousToolMode.Idx != m_modeIdx)
        {
            m_mode = TileDelaunayNavmeshToolMode.Values[m_modeIdx];
        }

        if (m_mode == TileDelaunayNavmeshToolMode.ADD_OBSTACLE)
        {
            ImGui.SliderFloat("Obstacle Size", ref _tool.m_obstacleSize, 1f, 20f, "%.2f");
            ImGui.SliderFloat("Obstacle Size Variance", ref _tool.m_obstacleSizeVariance, 0f, 6f, "%.2f");
        }
        else if (m_mode == TileDelaunayNavmeshToolMode.BUILD)
        {
            ImGui.SliderFloat("Cell Size", ref _tool.m_cellSize, 0.1f, 1.0f, "%.2f");
            ImGui.SliderInt("Tile Size", ref _tool.m_tileSize, 1, 100);

            if (ImGui.Button("Create All Tile"))
            {
                var geom = _sample.GetInputGeom();
                var bmin = geom.GetMeshBoundsMin();
                var bmax = geom.GetMeshBoundsMax();

                _graph = _tool.buildNavGraph(
                    new Vector3(bmin.X, bmin.Y, bmin.Z),
                    new Vector3(bmax.X, bmax.Y, bmax.Z));

                if (null == _graph)
                {
                    Logger.Error("build graph failed!");
                }
                else
                {
                    Logger.Information($"build graph success, tileXCount:{_graph.tileXCount}, tileZCount:{_graph.tileZCount}");
                }
            }
        }
    }

    public void OnSampleChanged()
    {
    }

    public void SetSample(DemoSample sample)
    {
        _sample = sample;
    }
}
