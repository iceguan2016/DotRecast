using System;
using System.Collections.Generic;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Recast.Demo.Draw;
using DotRecast.Recast.Toolset;
using ImGuiNET;
using Serilog;
using UnityEngine;
using Pathfinding.Crowds;
using Pathfinding.Util;
using SharpSteer2.Helpers;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.View;
using Pathfinding.Triangulation.Math;
using Pathfinding.Triangulation.Factories;
using Pathfinding.Triangulation.AI;
using Volatile;
using FixMath.NET;
using Pathfinding.Main;
using System.IO;
using System.Threading.Channels;
using DotRecast.Recast.Demo.UI;
using System.Collections.Immutable;

namespace DotRecast.Recast.Demo.Tools;

public class PhysicsWorldGizmosDrawer : IGizmosDrawer
{
    private DrawInterface _draw = null;

    public PhysicsWorldGizmosDrawer(DrawInterface draw)
    {
        this._draw = draw;
    }

    public void DrawArrow(VoltVector2 start, VoltVector2 end, VoltVector2 arrowSize, float lineWidth, Volatile.Color c)
    {
        var _height = _draw.GetMapHeight();
        _draw.DrawArrow(start.ToUnityVec3(_height), end.ToUnityVec3(_height), arrowSize.ToUnityVec2(), lineWidth, c.ToUnityColor());
    }

    public void DrawCircle(VoltVector2 p, float r, Volatile.Color c)
    {
        var _height = _draw.GetMapHeight();
        _draw.DrawCircle(p.ToUnityVec3(_height), r, c.ToUnityColor());
    }

    public void DrawCube(VoltVector2 p, VoltVector2 size, Volatile.Color c)
    {
        var _height = _draw.GetMapHeight();
        _draw.DrawCube(p.ToUnityVec3(_height),  size.ToUnityVec3(0.0f), c.ToUnityColor());
    }

    public void DrawLine(VoltVector2 a, VoltVector2 b, Volatile.Color c, float lineWidth = 1)
    {
        var _height = _draw.GetMapHeight();
        _draw.DrawLine(a.ToUnityVec3(_height),  b.ToUnityVec3(_height), c.ToUnityColor(), lineWidth);
    }

    public void DrawSolidCube(VoltVector2 p, Fix64 angle, VoltVector2 size, Volatile.Color c)
    {
        var _height = _draw.GetMapHeight();
        var rotation = UnityEngine.Quaternion.Euler(0.0f, (float)angle, 0.0f);
        _draw.DrawSolidCube(p.ToUnityVec3(_height), rotation, size.ToUnityVec3(0.0f), c.ToUnityColor());
    }

    public void DrawTriangle(VoltVector2 v0, VoltVector2 v1, VoltVector2 v2, Volatile.Color c)
    {
        var _height = _draw.GetMapHeight();
        _draw.DrawTriangle(v0.ToUnityVec3(_height),  v1.ToUnityVec3(_height), v1.ToUnityVec3(_height), c.ToUnityColor());
    }
}

public class TestFixedCrowdTool : IRcToolable
{
    // pseudo random generator
    Random rand = new Random((int)DateTime.Now.Ticks);

    public UnityEngine.Vector3? StartPoint { get; set; }
    public UnityEngine.Vector3? EndPoint { get; set; }

    public Face HitFace { get; set; }

    // pathfinder
    private PathFinder _pathfinder = null;
    public PathFinder Pathfinder { get { return _pathfinder; } }

    // obstacles
    private List<Obstacle> _obstacles = new List<Obstacle>();

    // crowd entity templates
    public TMovableEntityTemplate[] MovableEntityTemplates = new TMovableEntityTemplate[] {
        new TMovableEntityTemplate()
        {
            // 
            Radius = FixMath.F64.FromFloat(0.5f),
            // maximum move speed
            MaxSpeed = FixMath.F64.FromFloat(6.0f),
            // maximum force
            MaxForce = FixMath.F64.FromFloat(27.0f),
        
            //
            FollowPathAheadTime = FixMath.F64.FromFloat(3.0f),
            FollowPathWeight = FixMath.F64.FromFloat(1.0f),

            AvoidObstacleAheadTime = FixMath.F64.FromFloat(1.0f),
            AvoidObstacleWeight = FixMath.F64.FromFloat(0.3f),

            AvoidNeighborAheadTime = FixMath.F64.FromFloat(1.0f),
            AvoidNeighborWeight = FixMath.F64.FromFloat(1.0f),
        },
        new TMovableEntityTemplate()
        {
            // 
            Radius = FixMath.F64.FromFloat(1.5f),
            // maximum move speed
            MaxSpeed = FixMath.F64.FromFloat(3.0f),
            // maximum force
            MaxForce = FixMath.F64.FromFloat(54.0f),
        
            //
            FollowPathAheadTime = FixMath.F64.FromFloat(5.0f),
            FollowPathWeight = FixMath.F64.FromFloat(1.0f),

            AvoidObstacleAheadTime = FixMath.F64.FromFloat(2.0f),
            AvoidObstacleWeight = FixMath.F64.FromFloat(1.0f),

            PredictionAvoidIdleNeighborTime = FixMath.F64.FromFloat(0.2f),
            AvoidNeighborAheadTime = FixMath.F64.FromFloat(2.0f),
            AvoidNeighborWeight = FixMath.F64.FromFloat(1.0f),

            SeparationWeight = FixMath.F64.FromFloat(0.3f),
            AlignmentWeight = FixMath.F64.FromFloat(0.4f),
            CohesionWeight = FixMath.F64.FromFloat(0.1f),
        }       
    };
    private int _templateIndex = 0;

    // crowd entities
    private IMovableEntityManager _entityManager = null;
    public IMovableEntityManager EntityManager { get { return _entityManager; } }
    private List<UniqueId> _selectEntities = new List<UniqueId>();
    public List<UniqueId> SelectEntities { get { return _selectEntities; } }

    public Mesh Mesh 
    { 
        get 
        {
            return EntityManager.Map.NavMesh;
        } 
    }

    // 
    public bool IsOpenRecord = false;
    public float TerraintHeight = 0.0f;

    double RandomRange(double min, double max)
    {
        return min + rand.NextDouble() * (max - min);
    }

    public string GetName()
    {
        return "Test FixedCrowd Tool";
    }

    public void Start(double x, double y, double mapWidth, double mapHeight, double terrainHeight)
    {
        PathfindingMoudle.StartupModule();

        var h = FixMath.F64.FromDouble(terrainHeight);
        _entityManager = new MovableEntityManager();
        if (null != _entityManager)
        {
            var mapYExtent = FixMath.F64.FromDouble(5.0f);
            var param = new IMovableEntityManager.FInitializeParams()
            {
                MapBoundsMin = new FixMath.F64Vec3(FixMath.F64.FromDouble(x), h - mapYExtent / 2, FixMath.F64.FromDouble(y)),
                MapBoundsMax = new FixMath.F64Vec3(FixMath.F64.FromDouble(x + mapWidth), h + mapYExtent / 2, FixMath.F64.FromDouble(y + mapHeight)),
                MapCellDivs = 10,

                IsOpenRecord = IsOpenRecord,
                RecordRootDir = Debug.RecordRootDir,
            };

            if (_entityManager.Initialize(param))
            {
                AddRandomObstacle(30, param.MapBoundsMin, param.MapBoundsMax);

                _pathfinder = new PathFinder();
                _pathfinder.set_mesh(_entityManager.Map.NavMesh);
            }
        }
    }

    public void Stop() 
    {
        if (_entityManager != null)
        {
            _entityManager.StopRecord();
            _entityManager.StopReplay();
            _entityManager.UnInitialize();
            _entityManager = null;
        }

        PathfindingMoudle.ShutdownModule();
    }

    void AddRandomObstacle(int totalCount, FixMath.F64Vec3 boundsMin, FixMath.F64Vec3 boundsMax)
    {
        // populate mesh with many square objects
        for (int i = 0; i < totalCount; ++i)
        {
            var x = RandomRange(boundsMin.X.Double, boundsMax.X.Double);
            var y = RandomRange(boundsMin.Z.Double, boundsMax.Z.Double);

            AddObstacle(x, y);
        }  // show result mesh on screen
    }

    public void Destroy()
    {
        if (null != _entityManager)
        {
            _entityManager.UnInitialize();
        }

        PathfindingMoudle.ShutdownModule();
    }

    public void Update(double inDeltaTime)
    {
        if (null != _entityManager)
        {
            _entityManager.FrameBegin();
            _entityManager.Tick(FixMath.F64.FromDouble(inDeltaTime));
            _entityManager.FrameEnd();

            _entityManager.TickReplay(FixMath.F64.FromDouble(inDeltaTime));
        }
    }
    public void AddObstacle(double x, double y)
    {
        if (null == Mesh)
            return;
        
        var scaleX = FixMath.F64.FromDouble(RandomRange(10, 40) / 600.0f * Mesh._width.Float);
        var scaleY = FixMath.F64.FromDouble(RandomRange(10, 40) / 600.0f * Mesh._height.Float);
        var angle = FixMath.F64.FromDouble((RandomRange(0, 1000) / 1000) * Math.PI / 2);
        var posx = FixMath.F64.FromDouble(x);
        var posy = FixMath.F64.FromDouble(y);

        // 创建阻挡物实体对向
        var rotation = FixMath.F64Quat.FromYawPitchRoll(angle, FixMath.F64.Zero, FixMath.F64.Zero);
        //var cos = FixMath.F64.CosFast(angle);
        //var sin = FixMath.F64.SinFast(angle);
        var xHalfSize = scaleX;
        var yHalfSize = scaleY;
        var template = new TUnMovableEntityTemplate() 
        { 
            HalfExtent = new FixMath.F64Vec2(xHalfSize, yHalfSize),
        };

        var height = _entityManager.Map.TerrainHeight;
        var spawnPos = new FixMath.F64Vec3(posx, height, posy);
        var spawnRot = rotation;

        // 注册模板
        var tid = UniqueId.FromName($"UnMovable p:{spawnPos} r:{spawnRot} hash:{template.GetHashCode()}");
        if (null == _entityManager.FindTemplate(tid))
        {
            _entityManager.RegisterTemplate(tid, template);
        }

        // 创建怪物
        var param = new CreateEntityParams() 
        {
            SpawnPosition = spawnPos,
            SpawnRotation = rotation,

            TemplateId = tid,
        };
        var obstacleId = _entityManager.CreateEntity(param);
    }

    public void RemoveObstacle(Obstacle obstacle)
    {
        if (null == Mesh)
            return;
        Mesh.deleteObject(obstacle);
        _obstacles.Remove(obstacle);
    }

    public Obstacle HitObstacle(double x, double y)
    {
        return EntityManager.Map.HitObstacle(FixMath.F64.FromDouble(x), FixMath.F64.FromDouble(y));
    }

    // Crowd entity interface
    public UniqueId AddCrowdEntity(double x, double y, int groupId = 1)
    {
        var template = MovableEntityTemplates[_templateIndex];
        var tid = UniqueId.FromName($"Movable r:{template.Radius} hash:{template.GetHashCode()}");
        if (null == _entityManager.FindTemplate(tid))
        {
            _entityManager.RegisterTemplate(tid, template);
        }

        var terrainHeight = _entityManager.Map.TerrainHeight;
        var Params = new CreateEntityParams() 
        {
            EntityId = UniqueId.InvalidID,
            SpawnPosition = FixMath.F64Vec3.FromDouble(x, terrainHeight.Double, y),
            SpawnRotation = FixMath.F64Quat.Identity,

            TemplateId = tid,
            // PathwayQuerier = _entityManager.Map,
            // LocalBoundaryQuerier = _entityManager.Map,
            // AnnotationService = _annotationServerice,
        };

        var entityId = _entityManager.CreateEntity(Params);
        // 设置group
        var groupMask = 1 << groupId;
        var groupsToAvoid = 1 << groupId;
        _entityManager.SetEntityParams(entityId, null, null, null, groupMask, groupsToAvoid);
        return entityId;
    }

    public void RemoveCrowdEntity(UniqueId InEntityId)
    {
        _entityManager.DeleteEntity(InEntityId);
    }

    public UniqueId HitCrowdEntity(double x, double y)
    {
        var terrainHeight = _entityManager.Map.TerrainHeight;
        var hitPos = FixMath.F64Vec3.FromDouble(x, terrainHeight.Double, y);
        var hitEntityId = UniqueId.InvalidID;
        _entityManager.ForEachEntity((InEntity) =>
        {
            if (InEntity is MovableEntity entity)
            {
                var q = FixMath.F64Quat.LookRotation(entity.Forward, entity.Up);
                var local_p = FixMath.F64Quat.Inverse(q) * (hitPos - entity.Position);
                if (FixMath.F64.Abs(local_p.X) < entity.Radius && FixMath.F64.Abs(local_p.Z) < entity.Radius)
                {
                    hitEntityId = entity.ID;
                }
            }
        });
        return hitEntityId;
    }

    public void MoveCrowdEntity(double x, double y)
    {
        var terrainHeight = _entityManager.Map.TerrainHeight;
        var Position = FixMath.F64Vec3.FromDouble(x, terrainHeight.Double, y);

        for (var i = 0; i < _selectEntities.Count; ++i)
        {
            _entityManager.MoveEntity(_selectEntities[i], Position);
        }
    }

    public void DrawCrowdEntity()
    {
        if (null == _entityManager) 
            return;

        _entityManager.ForEachEntity((InEntity) =>
        {
            if (InEntity is MovableEntity entity)
            {
                if (Debug.IsSimulationMode(eSimulationMode.Normal)
                    || Debug.IsSimulationMode(eSimulationMode.Replay))
                {
                    if (IsDrawSelectionInfo)
                    {
                        var index = SelectEntities.FindIndex(Id => Id == entity.ID);
                        entity.OnDraw(index != -1);
                    }
                    else
                    {
                        entity.OnDraw(false);
                    }
                }
                else if (Debug.IsSimulationMode(eSimulationMode.Playback))
                {
                    if (entity.ID == Debug.DebugEntityId)
                    {
                        entity.Debuger?.Draw(entity.Annotation, entity, _entityManager.FrameNo, Debug.PlaybackFrameNo);
                    }
                }
            }
        });
    }
    // End

    // Pathfind test
    public int TestPathIterTimes = 1024;
    private List<FixMath.F64> _path = new List<FixMath.F64>(); 
    public void TestFindPath(double fromX, double fromY, double toX, double toY, double radius)
    {
        if (null == _pathfinder)
        {
            return;
        }

        _path.Clear();
        _pathfinder.findPath(
            FixMath.F64.FromDouble(fromX),
            FixMath.F64.FromDouble(fromY),
            FixMath.F64.FromDouble(toX),
            FixMath.F64.FromDouble(toY),
            FixMath.F64.FromDouble(radius),
            TestPathIterTimes,
            _path);
    }

    public void DrawTestFindPath(DrawInterfaceImplement draw, SimpleView view)
    {
        if (null == _pathfinder)
        {
            return;
        }

        var old = draw.MapHeight;

        // draw visited faces
        draw.MapHeight += 0.5f;
        for (var i=0; i<_pathfinder.listFaces.Count; ++i)
        {
            var face = _pathfinder.listFaces[i];
            DrawFace(face, view);
        }

        // draw path
        draw.MapHeight += 0.1f;
        if (_path.Count > 0)
        {
            var v0 = new UnityEngine.Vector3(_path[0].Float, draw.MapHeight, _path[1].Float);
            var PointSize = UnityEngine.Vector3.one * 0.2f;
            draw.DrawCube(v0, PointSize, UnityEngine.Color.red);
            for (var i = 2; i < _path.Count; i += 2)
            {
                var v1 = new UnityEngine.Vector3(_path[i].Float, draw.MapHeight, _path[i + 1].Float);
                draw.DrawLine(v0, v1, UnityEngine.Color.green);
                draw.DrawCube(v1, PointSize, UnityEngine.Color.red);
                v0 = v1;
            }
        }

        draw.MapHeight = old;
    }
    // end

    public void DrawFace(Face face, SimpleView view)
    {
        for (var i = 0; i < Mesh._faces.Count; ++i)
        {
            var f = Mesh._faces[i];
            if (f == face)
            {
                FromFaceToInnerEdges iterEdge = new FromFaceToInnerEdges();
                iterEdge.set_fromFace(face);
                while (true)
                {
                    var innerEdge = iterEdge.next();
                    if (innerEdge == null)
                    {
                        break;
                    }

                    view.drawEdge(innerEdge, UnityEngine.Color.magenta, 10.0f);
                }

                var color = UnityEngine.Color.yellow;
                color.a *= 0.1f;
                view.drawFace(face, color);
                break;
            }
        }
    }

    public void DrawEdge(Edge edge)
    {
        
    }

    // Selection
    public bool IsSelecting { get; private set; }
    public RcVec3f SelectionStartPosition { get; private set; }
    public RcVec3f SelectionEndPosition { get; private set; }
    public void BeginSelection(RcVec3f p)
    {
        IsSelecting = true;
        SelectionStartPosition = p;
    }

    public void UpdateSelection(RcVec3f p, bool finished)
    {
        if (!IsSelecting) return;
        SelectionEndPosition = p;
        if (finished)
        {
            IsSelecting = false;

            // 更新选中的单位
            var center = (SelectionStartPosition + SelectionEndPosition) * 0.5f;
            var size = SelectionEndPosition - SelectionStartPosition;
            var halfSizeX = Math.Abs(size.X) * 0.5f;
            var halfSizeZ = Math.Abs(size.Z) * 0.5f;
            if (halfSizeX < 0.01f || halfSizeZ < 0.01f)
            {
                return;
            }

            _selectEntities.Clear();
            _entityManager.ForEachEntity((InEntity) =>
            {
                if (InEntity is MovableEntity entity)
                {
                    var pos = entity.Position;
                    if (Math.Abs(pos.X.Float - center.X) < halfSizeX &&
                        Math.Abs(pos.Z.Float - center.Z) < halfSizeZ)
                    {
                        _selectEntities.Add(InEntity.ID);
                    }
                }
            });
        }
    }

    public bool IsDrawNavmeshGrpah = false;
    public bool IsDrawPhysicsWorld = true;
    public bool IsDrawContactInfo = false;
    public bool IsDrawSelectionInfo = false;

    public void Layout_Toggles()
    {
        // debug toggles
        for (var i = 0; i < TMovableEntityTemplate.DebugVec3Toggles.Length; ++i)
        {
            var type = (eDebugVec3Item)i;
            ImGui.Checkbox(type.ToString(), ref TMovableEntityTemplate.DebugVec3Toggles[i]);
        }
    }

    public void Layout()
    {
        var propertyChanged = false;

        var neiIndex = Debug.WatchNeighborIndex;
        if (ImGui.SliderInt($"Watch neighbor {neiIndex}", ref neiIndex, 0, 15))
        {
            Debug.WatchNeighborIndex = neiIndex;
        }

        ImGui.SliderInt($"Draw vo step {Debug._drawVOIndex}", ref Debug._drawVOIndex, 0, 15);

        ImGui.Text($"Movable Entity Templates");
        ImGui.Separator();

        var previousTemplateIndex = _templateIndex;
        for (var i = 0; i < MovableEntityTemplates.Length; ++i)
        {
            var template = MovableEntityTemplates[i];
            if (ImGui.RadioButton(string.Format("Radius - {0}", template.Radius), ref _templateIndex, i))
            {
                _templateIndex = i;
            }
        }
        ImGui.NewLine();

        // 模板参数
        Layout_Toggles();

        var currTemplate = MovableEntityTemplates[_templateIndex];

        var stopRadius = currTemplate.StopMoveRadius.Float;
        propertyChanged |= ImGui.SliderFloat("StopMoveRadius", ref stopRadius, 1.0f, 10.0f);
        currTemplate.StopMoveRadius = FixMath.F64.FromFloat(stopRadius);

        var radius = currTemplate.Radius.Float;
        propertyChanged |= ImGui.SliderFloat("Radius", ref radius, 0.1f, 5.0f);
        currTemplate.Radius = FixMath.F64.FromFloat(radius);

        var maxSpeed = currTemplate.MaxSpeed.Float;
        propertyChanged |= ImGui.SliderFloat("MaxSpeed", ref maxSpeed, 1.0f, 10.0f);
        currTemplate.MaxSpeed = FixMath.F64.FromFloat(maxSpeed);

        var maxForce = currTemplate.MaxForce.Float;
        propertyChanged |= ImGui.SliderFloat("MaxForce", ref maxForce, 10.0f, 100.0f);
        currTemplate.MaxForce = FixMath.F64.FromFloat(maxForce);

        var forwardMoveWeight = currTemplate.ForwardMoveWeight.Float;
        propertyChanged |= ImGui.SliderFloat("ForwardMoveWeight", ref forwardMoveWeight, 0.0f, 10.0f);
        currTemplate.ForwardMoveWeight = FixMath.F64.FromFloat(forwardMoveWeight);

        var followPathAheadTime = currTemplate.FollowPathAheadTime.Float;
        propertyChanged |= ImGui.SliderFloat("FollowPathAheadTime", ref followPathAheadTime, 0.1f, 5.0f);
        currTemplate.FollowPathAheadTime = FixMath.F64.FromFloat(followPathAheadTime);

        var followPathWeight = currTemplate.FollowPathWeight.Float;
        propertyChanged |= ImGui.SliderFloat("FollowPathWeight", ref followPathWeight, 0.0f, 5.0f);
        currTemplate.FollowPathWeight = FixMath.F64.FromFloat(followPathWeight);

        var predictionAvoidIldeNeighborime = currTemplate.PredictionAvoidIdleNeighborTime.Float;
        propertyChanged |= ImGui.SliderFloat("PredictionAvoidIdleNeighborTime", ref predictionAvoidIldeNeighborime, 0.0f, 5.0f);
        currTemplate.PredictionAvoidIdleNeighborTime = FixMath.F64.FromFloat(predictionAvoidIldeNeighborime);

        var avoidNeighborAheadTime = currTemplate.AvoidNeighborAheadTime.Float;
        propertyChanged |= ImGui.SliderFloat("AvoidNeighborAheadTime", ref avoidNeighborAheadTime, 0.1f, 5.0f);
        currTemplate.AvoidNeighborAheadTime = FixMath.F64.FromFloat(avoidNeighborAheadTime);

        var avoidNeighborWeight = currTemplate.AvoidNeighborWeight.Float;
        propertyChanged |= ImGui.SliderFloat("AvoidNeighborWeight", ref avoidNeighborWeight, 0.0f, 20.0f);
        currTemplate.AvoidNeighborWeight = FixMath.F64.FromFloat(avoidNeighborWeight);

        var avoidObstacleAheadTime = currTemplate.AvoidObstacleAheadTime.Float;
        propertyChanged |= ImGui.SliderFloat("AvoidObstacleAheadTime", ref avoidObstacleAheadTime, 0.1f, 5.0f);
        currTemplate.AvoidObstacleAheadTime = FixMath.F64.FromFloat(avoidObstacleAheadTime);

        var avoidObstacleWeight = currTemplate.AvoidObstacleWeight.Float;
        propertyChanged |= ImGui.SliderFloat("AvoidObstacleWeight", ref avoidObstacleWeight, 0.0f, 5.0f);
        currTemplate.AvoidObstacleWeight = FixMath.F64.FromFloat(avoidObstacleWeight);

        var separationWeight = currTemplate.SeparationWeight.Float;
        propertyChanged |= ImGui.SliderFloat("SeparationWeight", ref separationWeight, 0.0f, 10.0f);
        currTemplate.SeparationWeight = FixMath.F64.FromFloat(separationWeight);

        var alignmentWeight = currTemplate.AlignmentWeight.Float;
        propertyChanged |= ImGui.SliderFloat("AlignmentWeight", ref alignmentWeight, 0.0f, 10.0f);
        currTemplate.AlignmentWeight = FixMath.F64.FromFloat(alignmentWeight);

        var cohesionWeight = currTemplate.CohesionWeight.Float;
        propertyChanged |= ImGui.SliderFloat("CohesionWeight", ref cohesionWeight, 0.0f, 10.0f);
        currTemplate.CohesionWeight = FixMath.F64.FromFloat(cohesionWeight);

        if (propertyChanged)
        {
            EntityManager.ForEachEntity((InEntity) =>
            {
                if (InEntity is MovableEntity entity)
                {
                    entity.OnTemplatePropertyChanged();
                }
            });
        }
    }
}

public class TestFixedCrowdSampleTool : ISampleTool
{
    public static readonly ILogger Logger = Log.ForContext<TestNavmeshSampleTool>();

    private DemoSample _sample;

    private TestFixedCrowdTool _tool = null;

    private DrawInterfaceImplement _draw = null;
    private PhysicsWorldGizmosDrawer _physicsWorldDrawer = null;

    private SimpleView _view = null;

    private TestDaedalusToolMode m_mode = TestDaedalusToolMode.ADD_OBSTACLE;
    private int m_modeIdx = TestDaedalusToolMode.ADD_OBSTACLE.Idx;

    static RcVec3f DEFAULT_TERRAIN_HALF_SIZE = new RcVec3f(50.0f, 5.0f, 50.0f);

    public IRcToolable GetTool()
    {
        return _tool;
    }

    public void HandleClick(RcVec3f s, RcVec3f p, bool shift)
    {

    }

    bool GetRaycastHitPos(RcVec3f start, RcVec3f direction, out RcVec3f outHitPos)
    {
        // 平面 y = _draw.MapHeight
        outHitPos = new RcVec3f();
        
        var Mesh = _tool.EntityManager.Map.NavMesh;
        var PlaneHalfSize = 1000.0f;

        var src = start;
        var dst = start + direction * 100.0f;
        var bmin = new RcVec3f(-PlaneHalfSize, _draw.MapHeight, -PlaneHalfSize);
        var bmax = new RcVec3f(PlaneHalfSize, _draw.MapHeight, PlaneHalfSize);
        if (!RcIntersections.IsectSegAABB(src, dst, bmin, bmax, out var btmin, out var btmax))
        {
            return false;
        }

        outHitPos = src + (dst - src) * btmin;
        return true;
    }

    public void HandleClickRay(RcVec3f start, RcVec3f direction, bool shift)
    {
        if (null == _draw) return;
        var Mesh = _tool.EntityManager.Map.NavMesh;
        if (null == Mesh) return;

        if (!GetRaycastHitPos(start, direction, out var hitPos))
        {
            return;
        }

        Logger.Information($"HandleClickRay, hitPos:{hitPos}");

        Intersection loc = Geom2D.locatePosition(FixMath.F64.FromFloat(hitPos.X), FixMath.F64.FromFloat(hitPos.Z), _tool.Mesh);
        if (loc is Intersection_EVertex v)
        { 
        }
        else if (loc is Intersection_EEdge e)
        { 
        }
        else if (loc is Intersection_EFace f)
        {
            Logger.Information($"HandleClickRay, hit face:{f.face._id}");
            _tool.HitFace = f.face;
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

                _tool.TestFindPath(_tool.StartPoint.Value.x, _tool.StartPoint.Value.z, _tool.EndPoint.Value.x, _tool.EndPoint.Value.z, 0.5);

                //Logger.Information($"HandleClickRay, findPath length:{_tool.Path.length}");
            }
        }
        else if (m_mode == TestDaedalusToolMode.ADD_CROWD_ENTITY)
        {
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
                _tool.AddCrowdEntity(hitPos.X, hitPos.Z);
            }
        }
        else if (m_mode == TestDaedalusToolMode.SELECT_CROWD_ENTITY)
        {
            //var entityId = _tool.HitCrowdEntity(hitPos.X, hitPos.Z);

            //if (entityId != UniqueId.InvalidID)
            //{
            //    if (shift)
            //        _tool.SelectEntities.Remove(entityId);
            //    else
            //        _tool.SelectEntities.Add(entityId);
            //}
        }
        else if (m_mode == TestDaedalusToolMode.MOVE_CROWD_ENTITY)
        { 
            _tool.MoveCrowdEntity(hitPos.X, hitPos.Z);
        }
    }

    public void HandleSelectionRay(RcVec3f start, RcVec3f direction, bool finished)
    {
        if (m_mode == TestDaedalusToolMode.SELECT_CROWD_ENTITY)
        {
            if (!GetRaycastHitPos(start, direction, out var hitPos))
            {
                return;
            }

            if (!_tool.IsSelecting)
            {
                _tool.BeginSelection(hitPos);
            }

            _tool.UpdateSelection(hitPos, finished);
        }
    }

    public void HandleRender(NavMeshRenderer renderer)
    {
        var dd = renderer.GetDebugDraw();

        //var geom = _sample.GetInputGeom();
        //var bound_min = geom.GetMeshBoundsMin();
        //var bound_max = geom.GetMeshBoundsMax();
        var bound_min = new RcVec3f(-DEFAULT_TERRAIN_HALF_SIZE.X, -DEFAULT_TERRAIN_HALF_SIZE.Y, -DEFAULT_TERRAIN_HALF_SIZE.Z);
        var bound_max = new RcVec3f(DEFAULT_TERRAIN_HALF_SIZE.X, DEFAULT_TERRAIN_HALF_SIZE.Y, DEFAULT_TERRAIN_HALF_SIZE.Z);

        if (_draw == null && _tool.EntityManager != null)
        {
            _draw = new DrawInterfaceImplement(dd);

            _view = new SimpleView(_draw);
           
            if (null != _tool.EntityManager)
            {
                _tool.EntityManager.AnnotationService = new EntityAnnotationServerice(_draw);
            }

            _physicsWorldDrawer = new PhysicsWorldGizmosDrawer(_draw);
        }

        // 同步地形高度
        if (_draw != null && _tool.EntityManager != null)
        {
            _draw.MapHeight = _tool.EntityManager.Map.TerrainHeight.Float;
        }

        // draw bounds
        dd.DebugDrawBoxWire(
            bound_min.X, bound_min.Y - 3.0f, bound_min.Z,
            bound_max.X, bound_max.Y + 3.0f, bound_max.Z,
            DebugDraw.DuRGBA(255, 255, 255, 128), 1.0f);

        // draw graph mesh
        if (_tool.IsDrawNavmeshGrpah && null != _tool.Mesh) 
        {
            _view.drawMesh(_tool.Mesh);
        }

        if (m_mode == TestDaedalusToolMode.PATH_FINDER)
        {
            var CubeSize = new UnityEngine.Vector3(1.0f, 2.0f, 1.0f);
            if (null != _tool.StartPoint)
                _draw.DrawCube(_tool.StartPoint.Value, CubeSize, UnityEngine.Color.gray);
            if (null != _tool.EndPoint)
                _draw.DrawCube(_tool.EndPoint.Value, CubeSize, UnityEngine.Color.yellow);

            // draw path
            //if (_tool.Path.length > 0)
            //{
            //    var v0 = new UnityEngine.Vector3((float)_tool.Path[0], _draw.MapHeight, (float)_tool.Path[1]);
            //    var PointSize = UnityEngine.Vector3.one * 0.2f;
            //    _draw.DrawCube(v0, PointSize, UnityEngine.Color.red);
            //    for (var i = 2; i < _tool.Path.length; i += 2)
            //    {
            //        var v1 = new UnityEngine.Vector3((float)_tool.Path[i], _draw.MapHeight, (float)_tool.Path[i + 1]);
            //        _draw.DrawLine(v0, v1, UnityEngine.Color.green);
            //        _draw.DrawCube(v1, PointSize, UnityEngine.Color.red);
            //        v0 = v1;
            //    }
            //}

            _tool.DrawTestFindPath(_draw, _view);

            // draw face
            _tool.DrawFace(_tool.HitFace, _view);
        }
        else if (m_mode == TestDaedalusToolMode.SELECT_CROWD_ENTITY)
        {
            if (_tool.IsSelecting)
            {
                var p = (_tool.SelectionStartPosition + _tool.SelectionEndPosition) * 0.5f;
                var size = _tool.SelectionEndPosition - _tool.SelectionStartPosition;
                _draw.DrawSolidCube(new Vector3(p.X, p.Y, p.Z), Quaternion.identity, new Vector3(Math.Abs(size.X), 0.01f, Math.Abs(size.Z)), new UnityEngine.Color(1.0f, 0.0f, 0.0f, 0.3f));
                // _draw.DrawCircle(new Vector3(p.X, p.Y, p.Z), size.X)
            }
        }

        // draw crowd entities
        _tool.DrawCrowdEntity();

        // draw physics body
        Volatile.Color edgeColor = Volatile.Color.white;
        Volatile.Color normalColor = Volatile.Color.green;
        Volatile.Color bodyOriginColor = Volatile.Color.blue;
        Volatile.Color shapeOriginColor = Volatile.Color.yellow;
        Volatile.Color bodyAabbColor = Volatile.Color.magenta;
        Volatile.Color shapeAabbColor = Volatile.Color.cyan;

        if (_tool.IsDrawPhysicsWorld && null != _tool.EntityManager)
        {
            _tool.EntityManager.ForEachEntity((InEntity) =>
            {
                if (null != InEntity && null != InEntity.PhysicsBody)
                {
                    InEntity.PhysicsBody.GizmoDraw(_physicsWorldDrawer, edgeColor, normalColor, bodyOriginColor, shapeOriginColor, bodyAabbColor, shapeAabbColor, FixMath.NET.Fix64.One);
                }
            });
        }

        // draw contact pairs
        if (_tool.IsDrawContactInfo)
        {
            _tool.EntityManager.ForEachContactPair((EntityIDA, EntityIDB)  => { 
                var EntityA = _tool.EntityManager.GetEntityById(EntityIDA);
                if (null != EntityA && null != EntityA.PhysicsBody)
                {
                    EntityA.PhysicsBody.GizmoDraw(_physicsWorldDrawer, edgeColor, normalColor, bodyOriginColor, shapeOriginColor, bodyAabbColor, shapeAabbColor, FixMath.NET.Fix64.One);
                }
                
                var EntityB = _tool.EntityManager.GetEntityById(EntityIDB);
                if (null != EntityB && null != EntityB.PhysicsBody)
                {
                    EntityB.PhysicsBody.GizmoDraw(_physicsWorldDrawer, edgeColor, normalColor, bodyOriginColor, shapeOriginColor, bodyAabbColor, shapeAabbColor, FixMath.NET.Fix64.One);
                }
            });
        }

        // draw world axes
        if (null != _draw)
        {
            var o = Vector3.zero; o.y += 3.0f;
            var x = o + Vector3.right * 5;
            var y = o + Vector3.up * 5;
            var z = o + Vector3.forward * 5;
            _draw.DrawArrow(o, x, new Vector2(0.0f, 0.2f), 1.0f, UnityEngine.Color.red);
            _draw.DrawArrow(o, y, new Vector2(0.0f, 0.2f), 1.0f, UnityEngine.Color.green);
            _draw.DrawArrow(o, z, new Vector2(0.0f, 0.2f), 1.0f, UnityEngine.Color.blue);
        }
    }

    public void HandleUpdate(float dt)
    {
        if (Debug.CanNextStep() && null != _tool)
        {
            _tool.Update(dt);
        }
    }

    enum eUpdatePlaybackFrameReason
    {
        Reset = 0,
        Prev,
        Next,
    };

    void Layout_Toggles()
    {
        ImGui.Checkbox("Draw Navmesh Graph", ref _tool.IsDrawNavmeshGrpah);
        ImGui.Checkbox("Draw Physics World", ref _tool.IsDrawPhysicsWorld);
        ImGui.Checkbox("Draw Contact Info", ref _tool.IsDrawContactInfo);
        ImGui.Checkbox("Draw Selection Info", ref _tool.IsDrawSelectionInfo);
    }

    void Layout_TickControl()
    {
        var isPaused = Debug.IsPaused();
        if (ImGui.Button(isPaused ? ">" : "||"))
        {
            if (isPaused)
                Debug.Resume();
            else
                Debug.Pause();
        }
        ImGui.SameLine();
        if (ImGui.Button("|>"))
        {
            Debug.NextStep();
        }
        ImGui.NewLine();
    }

    void Layout_CreateEntityManager()
    {
        if (null == _sample || null == _tool)
            return;


        ImGui.Checkbox("Open Record", ref _tool.IsOpenRecord);
        ImGui.DragFloat("Terrain Height", ref _tool.TerraintHeight);

        if (ImGui.Button("Create Entity Manager"))
        {
            var bound_min = new RcVec3f(-DEFAULT_TERRAIN_HALF_SIZE.X, -DEFAULT_TERRAIN_HALF_SIZE.Y, -DEFAULT_TERRAIN_HALF_SIZE.Z);
            var bound_max = new RcVec3f(DEFAULT_TERRAIN_HALF_SIZE.X, DEFAULT_TERRAIN_HALF_SIZE.Y, DEFAULT_TERRAIN_HALF_SIZE.Z);

            var bound_center = (bound_min + bound_max) * 0.5f;

            var mapWidth = bound_max.X - bound_min.X;
            var mapHeight = bound_max.Z - bound_min.Z;

            RandomHelpers.InitialRandom((int)DateTime.Now.Ticks);

            if (null != _tool)
            {
                _tool.Start(0.0, 0.0, mapWidth, mapHeight, _tool.TerraintHeight);
                if (null != _draw)
                {
                    _tool.EntityManager.AnnotationService = new EntityAnnotationServerice(_draw);
                }
            }

            Logger.Information($"init graph mesh, mapWidth:{mapWidth}, mapHeight:{mapHeight}");
        }
    }

    void Layout_Replay()
    {
        if (!_tool.EntityManager.IsReplaying())
        {
            // 列出最新的录像文件
            //var fileList = Debug.GetLatestRecordFiles(5);
            //if (fileList != null)
            //{
            //    var CurrItemIndex = fileList.FindIndex(file => file.Name == Debug.ReplayFileName);

            //    if (ImGui.BeginCombo("Files", CurrItemIndex != -1 ? fileList[CurrItemIndex].Name : ""))
            //    {
            //        for (int n = 0; n < fileList.Count; n++)
            //        {
            //            bool is_selected = (CurrItemIndex == n);
            //            if (ImGui.Selectable(fileList[n].Name, is_selected))
            //                CurrItemIndex = n;
            //            if (is_selected)
            //                ImGui.SetItemDefaultFocus();   // 设置选中项为默认焦点
            //        }
            //    }
            //    ImGui.EndCombo();

            //    if (CurrItemIndex != -1)
            //    {
            //        Debug.ReplayFileName = fileList[CurrItemIndex].Name;
            //    }
            //    else
            //    {
            //        Debug.ReplayFileName = "";
            //    }
            //}

            ImGui.Text("Input Replay File");
            ImGui.Separator();

            const string strLoadReplayFile = "Load Replay File...";
            if (ImGui.Button(strLoadReplayFile))
            {
                ImGui.OpenPopup(strLoadReplayFile);
            }

            bool loadReplayFilePopup = true;
            if (ImGui.BeginPopupModal(strLoadReplayFile, ref loadReplayFilePopup, ImGuiWindowFlags.NoTitleBar))
            {
                var picker = ImFilePicker.GetFilePicker(strLoadReplayFile, Path.Combine(Environment.CurrentDirectory), ".record");
                if (picker.Draw())
                {
                    Debug.RecordRootDir = picker.CurrentFolder;
                    Debug.ReplayFileName = picker.SelectedFile;
                    ImFilePicker.RemoveFilePicker(strLoadReplayFile);
                }

                ImGui.EndPopup();
            }

            ImGui.LabelText("Replay File", Debug.ReplayFileName);

            if (ImGui.Button("StartReplay"))
            {
                if (Debug.ReplayFileName != "")
                {
                    _tool.EntityManager.StartReplay(Debug.ReplayFileName);
                }
            }
        }
        else
        {
            if (ImGui.Button("StopReplay"))
            {
                _tool.EntityManager.StopReplay();
            }

            if (_tool.EntityManager.IsPauseReplay)
            {
                if (ImGui.Button("ResumeReplay"))
                {
                    _tool.EntityManager.IsPauseReplay = false;
                }
            }
            else
            {
                if (ImGui.Button("PauseReplay"))
                {
                    _tool.EntityManager.IsPauseReplay = true;
                }
            }

            ImGui.LabelText("ReplaySpeed", $"{_tool.EntityManager.ReplaySpeed}");

            var currFrame = _tool.EntityManager.Recorder.CurrReplayFrame;
            var maxFrame = _tool.EntityManager.Recorder.MaxReplayFrame;
            ImGui.LabelText("ReplayFrame", $"{currFrame}/{maxFrame}");

            if (ImGui.Button("x0.5"))
                _tool.EntityManager.ReplaySpeed = FixMath.F64.Half;
            if (ImGui.Button("x1.0"))
                _tool.EntityManager.ReplaySpeed = FixMath.F64.One;
            if (ImGui.Button("x2.0"))
                _tool.EntityManager.ReplaySpeed = FixMath.F64.FromDouble(2.0);
            if (ImGui.Button("x4.0"))
                _tool.EntityManager.ReplaySpeed = FixMath.F64.FromDouble(4.0);
            if (ImGui.Button("x8.0"))
                _tool.EntityManager.ReplaySpeed = FixMath.F64.FromDouble(8.0);
        }
    }

    void Layout_TestFindPath()
    {
        if (ImGui.InputInt("MaxIterTimes", ref _tool.TestPathIterTimes, 1, 10))
        {
            if (null != _tool.StartPoint && null != _tool.EndPoint)
            {
                _tool.TestFindPath(_tool.StartPoint.Value.x, _tool.StartPoint.Value.z, _tool.EndPoint.Value.x, _tool.EndPoint.Value.z, 0.5);
            }
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
        ImGui.RadioButton(TestDaedalusToolMode.SELECT_CROWD_ENTITY.Label, ref m_modeIdx, TestDaedalusToolMode.SELECT_CROWD_ENTITY.Idx);
        ImGui.RadioButton(TestDaedalusToolMode.MOVE_CROWD_ENTITY.Label, ref m_modeIdx, TestDaedalusToolMode.MOVE_CROWD_ENTITY.Idx);
        ImGui.NewLine();

        if (previousToolMode.Idx != m_modeIdx)
        {
            m_mode = TestDaedalusToolMode.Values[m_modeIdx];
        }

        // 
        int simModeIndex = (int)Debug.GetSimlationMode();
        ImGui.RadioButton(eSimulationMode.Normal.ToString(), ref simModeIndex, (int)eSimulationMode.Normal);
        ImGui.RadioButton(eSimulationMode.Playback.ToString(), ref simModeIndex, (int)eSimulationMode.Playback);
        ImGui.RadioButton(eSimulationMode.Replay.ToString(), ref simModeIndex, (int)eSimulationMode.Replay);
        
        ImGui.NewLine();

        // 切换模式，重置EntityManager
        if (simModeIndex != (int)Debug.GetSimlationMode())
        {
            if (null != _tool)
            {
                _tool.Stop();
            }

            Debug.SetSimlationMode((eSimulationMode)simModeIndex);
        }

        if (m_mode == TestDaedalusToolMode.PATH_FINDER)
        {
            Layout_TestFindPath();
        }

        if (Debug.IsSimulationMode(eSimulationMode.Normal))
        {
            if (null == _tool.EntityManager)
            {
                Layout_CreateEntityManager();
            }
            else
            {
                Layout_Toggles();
                _tool.Layout();
                Layout_TickControl();
            }
        }
        else if (Debug.IsSimulationMode(eSimulationMode.Playback))
        {
            System.Func<int, eUpdatePlaybackFrameReason, int> UpdatePlaybackFrameNo = (FrameNo, Reason) =>
            {
                if (Debug.DebugEntityId.IsValid())
                {
                    var Entity = _tool.EntityManager.GetEntityById(Debug.DebugEntityId) as MovableEntity;
                    if (null != Entity)
                    {
                        if (Reason == eUpdatePlaybackFrameReason.Reset || 
                            Reason == eUpdatePlaybackFrameReason.Prev)
                        {
                            return Entity.Debuger.GetPrevValidDataFrame(FrameNo, Debug.DebugerDataType);
                        }
                        else if (Reason == eUpdatePlaybackFrameReason.Next)
                        {
                            return Entity.Debuger.GetNextValidDataFrame(FrameNo, Debug.DebugerDataType);
                        }
                    }
                }
                return FrameNo;
            };

            if (null != _tool.EntityManager)
            {
                var FrameNo = _tool.EntityManager.FrameNo;
                ImGui.NewLine();
                ImGui.LabelText("Frame", string.Format("{0}-{1}", FrameNo, Debug.PlaybackFrameNo));
                ImGui.NewLine();
                if (ImGui.Button("Reset"))
                    Debug.PlaybackFrameNo = UpdatePlaybackFrameNo(FrameNo, eUpdatePlaybackFrameReason.Reset);
                ImGui.SameLine();
                if (ImGui.Button("Prev"))
                    Debug.PlaybackFrameNo = UpdatePlaybackFrameNo(Math.Max(Debug.PlaybackFrameNo - 1, 1), eUpdatePlaybackFrameReason.Prev);
                ImGui.SameLine();
                if (ImGui.Button("Next"))
                    Debug.PlaybackFrameNo = UpdatePlaybackFrameNo(Math.Min(Debug.PlaybackFrameNo + 1, FrameNo), eUpdatePlaybackFrameReason.Next);
                ImGui.NewLine();


                // 显示调试信息类型
                {
                    var CurrItemIndex = (int)Debug.DebugerDataType;
                    var DataTypes = System.Enum.GetNames(typeof(MovableEntityDebuger.eDebugerDataType));

                    if (ImGui.BeginCombo("DataType", DataTypes[CurrItemIndex]))
                    {
                        for (int n = 0; n < DataTypes.Length; n++)
                        {
                            bool is_selected = (CurrItemIndex == n);
                            if (ImGui.Selectable(DataTypes[n], is_selected))
                                CurrItemIndex = n;
                            if (is_selected)
                                ImGui.SetItemDefaultFocus();   // 设置选中项为默认焦点
                        }
                    }
                    ImGui.EndCombo();

                    if (CurrItemIndex != -1)
                    {
                        var Type = (MovableEntityDebuger.eDebugerDataType)CurrItemIndex;
                        if (Debug.DebugerDataType != Type)
                        {
                            Debug.DebugerDataType = Type;
                            Debug.PlaybackFrameNo = UpdatePlaybackFrameNo(Debug.PlaybackFrameNo, eUpdatePlaybackFrameReason.Reset);
                        }
                    }
                }

                // 显示所有EntityId
                var Items = new List<string>();
                _tool.EntityManager.ForEachEntity(entity =>
                {
                    Items.Add(entity.ID.Id.ToString());
                });

                if (Items.Count > 0)
                {
                    var DebugEntityId = Debug.DebugEntityId.Id.ToString();
                    var CurrItemIndex = Items.FindIndex(id => id == DebugEntityId);
                    if (ImGui.BeginCombo("EntityId", CurrItemIndex != -1 ? Items[CurrItemIndex] : "None"))
                    {
                        for (int n = 0; n < Items.Count; n++)
                        {
                            bool is_selected = (CurrItemIndex == n);
                            if (ImGui.Selectable(Items[n], is_selected))
                                CurrItemIndex = n;
                            if (is_selected)
                                ImGui.SetItemDefaultFocus();   // 设置选中项为默认焦点
                        }
                    }
                    ImGui.EndCombo();

                    if (CurrItemIndex != -1)
                    {
                        var id = (uint)int.Parse(Items[CurrItemIndex]);
                        if (Debug.DebugEntityId.Id != id)
                        {
                            Debug.DebugEntityId = new UniqueId(id, "");
                            Debug.PlaybackFrameNo = UpdatePlaybackFrameNo(FrameNo, eUpdatePlaybackFrameReason.Reset);
                        }
                    }
                    else
                    {
                        Debug.PlaybackFrameNo = -1;
                    }
                }
            }
            
        }
        else if (Debug.IsSimulationMode(eSimulationMode.Replay))
        {
            if (null == _tool.EntityManager)
            {
                Layout_CreateEntityManager();
            }
            else
            {
                Layout_Toggles();
                Layout_TickControl();
                _tool.Layout_Toggles();
                Layout_Replay();
            }
        }
    }

    public void OnSampleChanged()
    {
    }

    public void SetSample(DemoSample sample)
    {
        _sample = sample;

        if (null != _tool) _tool.Destroy();

        _tool = new TestFixedCrowdTool();

        // 默认不设置Mesh
        //_sample.Update(null, ImmutableArray<RcBuilderResult>.Empty, null);
    }
}
