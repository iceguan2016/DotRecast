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

    public string GetName()
    {
        return "Tile Delaunay Navmesh";
    }

    public BoxShape addShape(Vector3 pos, Quaternion rot, Vector3 extent)
    { 
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
        return null;
    }

    public List<BoxShape> getAllShapes()
    {
        return m_shapes;
    }

    public FNavGraph buildNavGraph()
    {
        FTiledNavmeshBuilder builder = new FTiledNavmeshBuilder();

        FDebugParams debugParams = new FDebugParams();
        FTiledNavmeshBuilder.FTiledNavmeshBuilderParams builderParams = new FTiledNavmeshBuilder.FTiledNavmeshBuilderParams();


        builder.Build(builderParams, debugParams);
        // _tool.BuildAllTiles(geom, settings, navMesh);
        return null;
    }
}

public class TileDelaunayNavmeshTool : ISampleTool
{
    private static readonly ILogger Logger = Log.ForContext<TileDelaunayNavmeshTool>();

    private DemoSample _sample;

    private FNavGraph _graph;

    private TileDelaunayTool _tool = new TileDelaunayTool();

    private TileDelaunayNavmeshToolMode m_mode = TileDelaunayNavmeshToolMode.ADD_OBSTACLE;
    private int m_modeIdx = TileDelaunayNavmeshToolMode.ADD_OBSTACLE.Idx;

    private float m_obstacleSize = 1.0f;
    private float m_obstacleSizeVariance = 0.0f;

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
                var rand = new RcRand(DateTime.Now.Ticks);

                var pos = new Vector3(p.X, p.Y, p.Z);
                var angle = rand.Next() * 360 * Mathf.Deg2Rad;
                var rot = Quaternion.AxisAngle(Vector3.up, angle);
                var extent = new Vector3(
                    m_obstacleSize + rand.Next() * m_obstacleSizeVariance,
                    m_obstacleSize + rand.Next() * m_obstacleSizeVariance,
                    m_obstacleSize + rand.Next() * m_obstacleSizeVariance
                 );
                var shape = _tool.addShape(pos, rot, extent);
                if (null == shape)
                { 
                    Logger.Error("add obstacle failed!");
                }
                else 
                {
                    Logger.Information($"add obstacle success, pos:{pos}, rot:{rot}, extent:{extent}");
                }
            }
        }
    }

    public void HandleClickRay(RcVec3f start, RcVec3f direction, bool shift)
    {
    }

    public void HandleRender(NavMeshRenderer renderer)
    {
        var obstacles = _tool.getAllShapes();

        for (int i = 0; i < obstacles.Count; i++) 
        {
            var shape = obstacles[i];
            shape.DrawGizmos();
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
            ImGui.SliderFloat("Obstacle Size", ref m_obstacleSize, 1f, 20f, "%.2f");
            ImGui.SliderFloat("Obstacle Size Variance", ref m_obstacleSizeVariance, 0f, 6f, "%.2f");
        }
        else if (m_mode == TileDelaunayNavmeshToolMode.BUILD)
        {
            if (ImGui.Button("Create All Tile"))
            {
                _graph = _tool.buildNavGraph();
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
