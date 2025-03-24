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
using Object = Pathfinding.Triangulation.Data.Object;
using Pathfinding.Triangulation.Factories;
using Pathfinding.Triangulation.AI;
using Volatile;
using FixMath.NET;

namespace DotRecast.Recast.Demo.Tools;

public class PhysicsWorldGizmosDrawer : IGizmosDrawer
{
    private DrawInterface _draw = null;
    private float _height = 0.0f;

    public PhysicsWorldGizmosDrawer(DrawInterface draw)
    {
        this._draw = draw;
        this._height = _draw.GetMapHeight() + 0.5f;
    }

    public void DrawArrow(VoltVector2 start, VoltVector2 end, VoltVector2 arrowSize, float lineWidth, Volatile.Color c)
    {
        _draw.DrawArrow(start.ToUnityVec3(_height), end.ToUnityVec3(_height), arrowSize.ToUnityVec2(), lineWidth, c.ToUnityColor());
    }

    public void DrawCircle(VoltVector2 p, float r, Volatile.Color c)
    {
        _draw.DrawCircle(p.ToUnityVec3(_height), r, c.ToUnityColor());
    }

    public void DrawCube(VoltVector2 p, VoltVector2 size, Volatile.Color c)
    {
        _draw.DrawCube(p.ToUnityVec3(_height),  size.ToUnityVec3(0.0f), c.ToUnityColor());
    }

    public void DrawLine(VoltVector2 a, VoltVector2 b, Volatile.Color c, float lineWidth = 1)
    {
        _draw.DrawLine(a.ToUnityVec3(_height),  b.ToUnityVec3(_height), c.ToUnityColor(), lineWidth);
    }

    public void DrawSolidCube(VoltVector2 p, Fix64 angle, VoltVector2 size, Volatile.Color c)
    {
        var rotation = UnityEngine.Quaternion.Euler(0.0f, (float)angle, 0.0f);
        _draw.DrawSolidCube(p.ToUnityVec3(_height), rotation, size.ToUnityVec3(0.0f), c.ToUnityColor());
    }

    public void DrawTriangle(VoltVector2 v0, VoltVector2 v1, VoltVector2 v2, Volatile.Color c)
    {
        _draw.DrawTriangle(v0.ToUnityVec3(_height),  v1.ToUnityVec3(_height), v1.ToUnityVec3(_height), c.ToUnityColor());
    }
}

public class TestFixedCrowdTool : IRcToolable, IPathwayQuerier, ILocalBoundaryQuerier
{
    // pseudo random generator
    Random rand = new Random((int)DateTime.Now.Ticks);

    // Graph mesh
    public Mesh Mesh { get; private set; }

    public UnityEngine.Vector3? StartPoint { get; set; }
    public UnityEngine.Vector3? EndPoint { get; set; }

    public Face HitFace { get; set; }

    // map height
    public FixMath.F64 MapHeight { get; set; }

    // pathfinder
    public PathFinder Pathfinder { get; private set; }

    // obstacles
    private List<Object> _obstacles = new List<Object>();

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
            AvoidObstacleWeight = FixMath.F64.FromFloat(1.0f),

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

            AvoidNeighborAheadTime = FixMath.F64.FromFloat(2.0f),
            AvoidNeighborWeight = FixMath.F64.FromFloat(1.0f),
        }
    };
    private int _templateIndex = 0;

    // crowd entities
    private IMovableEntityManager _entityManager = new MovableEntityManager();
    public IMovableEntityManager EntityManager { get { return _entityManager; } }
    private List<UniqueId> _selectEntities = new List<UniqueId>();
    public List<UniqueId> SelectEntities { get { return _selectEntities; } }

    // annotation
    private EntityAnnotationServerice _annotationServerice = null;
    public DrawInterface DrawInterface 
    { 
        set 
        {
            _annotationServerice = new EntityAnnotationServerice(value);
        } 
    }

    double RandomRange(double min, double max)
    {
        return min + rand.NextDouble() * (max - min);
    }

    public string GetName()
    {
        return "Test FixedCrowd Tool";
    }

    public void Start()
    {
        if (null != _entityManager)
        {
            _entityManager.Initialize();
        }
    }

    public void Destroy()
    {
        if (null != _entityManager)
        {
            _entityManager.UnInitialize();
        }
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
        if (null == Mesh)
            return;
        var hxObject = new Object();
        var shapeCoords = new List<FixMath.F64> {
                            -FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One,
                             FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One, FixMath.F64.One,
                             FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One, FixMath.F64.One,
                            -FixMath.F64.One, FixMath.F64.One, -FixMath.F64.One, -FixMath.F64.One };

        hxObject._coordinates = shapeCoords;
        hxObject._scaleX = FixMath.F64.FromDouble(RandomRange(10, 40) / 600.0f * Mesh._width.Float);
        hxObject._scaleY = FixMath.F64.FromDouble(RandomRange(10, 40) / 600.0f * Mesh._height.Float);
        //hxObject._scaleX = infos[i * 5 + 2];
        //hxObject._scaleY = infos[i * 5 + 3];
        hxObject._rotation = FixMath.F64.FromDouble((RandomRange(0, 1000) / 1000) * Math.PI / 2);
        //hxObject._rotation = infos[i * 5 + 4];
        hxObject._x = FixMath.F64.FromDouble(x);
        hxObject._y = FixMath.F64.FromDouble(y);
        //hxObject._x = infos[i * 5 + 0];
        //hxObject._y = infos[i * 5 + 1];

        // Debug.LogToFile($"BuildFixedGraphMesh pos:({hxObject._x}, {hxObject._y}), scale:({hxObject._scaleX}, {hxObject._scaleY}), rotation:{hxObject._rotation}");

        Mesh.insertObject(hxObject);

        // 添加到物理层
        var angle = hxObject._rotation;
        var cos = FixMath.F64.CosFast(angle);
        var sin = FixMath.F64.SinFast(angle);
        var xHalfSize = hxObject._scaleX / 2;
        var yHalfSize = hxObject._scaleY / 2;
        var template = new TUnMovableEntityTemplate() 
        { 
            DirU = new FixMath.F64Vec2(cos, sin),
            DirV = new FixMath.F64Vec2(-sin, cos),
            HalfExtent = new FixMath.F64Vec2(xHalfSize, yHalfSize),
        };

        var param = new CreateEntityParams() 
        {
            SpawnPosition = new FixMath.F64Vec3(hxObject._x, MapHeight, hxObject._y),
            SpawnRotation = FixMath.F64Quat.Identity,

            Template = template,
        };
        _entityManager.CreateEntity(param);

        _obstacles.Add(hxObject);
    }

    public void RemoveObstacle(Object obstacle)
    {
        if (null == Mesh)
            return;
        Mesh.deleteObject(obstacle);
        _obstacles.Remove(obstacle);
    }

    public Object HitObstacle(double x, double y)
    {
        var p = FixMath.F64Vec2.FromDouble(x, y);

        for (var i = 0; i < _obstacles.Count; ++i)
        {
            var o = _obstacles[i];
            var edges = _obstacles[i].get_edges();
            if (edges.Count == 0)
                continue;

            // 
            var matrix = new Matrix2D();
            matrix.identity();
            matrix.translate(-o._x, -o._y);
            matrix.rotate(-o.get_rotation());

            var local_p = p;
            matrix.tranform(local_p);

            if (FixMath.F64.Abs(local_p.X) < o._scaleX && FixMath.F64.Abs(local_p.Y) < o._scaleY)
            {
                return o;
            }
        }
        return null;
    }

    public bool BuildGraphMesh(FixMath.F64 mapWidth, FixMath.F64 mapHeight)
    {
        // build a rectangular 2 polygons mesh of mapWidth x mapHeight
        Mesh = RectMesh.buildRectangle(mapWidth, mapHeight);
        if (null == Mesh)
        {
            return false;
        }

        // populate mesh with many square objects
        for (int i = 0; i < 30; ++i)
        {
            var x = RandomRange(50, 600) / 600.0f * mapWidth.Float;
            var y = RandomRange(50, 600) / 600.0f * mapHeight.Float;
            
            AddObstacle(x, y);
        }  // show result mesh on screen

        // pathfinder
        Pathfinder = new PathFinder();
        Pathfinder.set_mesh(Mesh);

        return true;
    }

    // IPathwayQuerier interface
    public virtual SharpSteer2.Pathway.PolylinePathway FindPath(SharpSteer2.IVehicle vehicle, FixMath.F64Vec3 target)
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
    // End

    // ILocalBoundaryQuerier interface
    public virtual int QueryBoundaryInCircle(SharpSteer2.IVehicle vehicle, FixMath.F64 inRadius, BoundarySegement[] outResults)
    {
        if (null == outResults) return 0;
        var center = vehicle.PredictFuturePosition(FixMath.F64.Zero);
        var loc = Geom2D.locatePosition(center.X, center.Z, Mesh);
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

        System.Func<hxDaedalus.data.Vertex, FixMath.F64Vec3> VertextToF64Vec3 = (vertex) => {
            var pos = vertex.get_pos();
            return FixMath.F64Vec3.FromDouble(pos.x, MapHeight.Double, pos.y);
        };

        System.Func<Face, bool> CheckIntersectionFaceAndCircle = (face) => {
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
                        if (outterEdge == null) continue;
                        var adjacentFace = outterEdge.get_leftFace();
                        if (adjacentFace == null) continue;
                        // 检查是否已经访问过
                        if (visiedFaces.Contains(adjacentFace)) continue;
                        // 检查是否在范围内
                        if (!CheckIntersectionFaceAndCircle(adjacentFace)) continue;
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
    // End

    // Crowd entity interface
    public UniqueId AddCrowdEntity(double x, double y)
    {
        var Params = new CreateEntityParams() 
        {
            EntityId = UniqueId.InvalidID,
            SpawnPosition = FixMath.F64Vec3.FromDouble(x, MapHeight.Double, y),
            SpawnRotation = FixMath.F64Quat.Identity,

            Template = MovableEntityTemplates[_templateIndex],
            PathwayQuerier = this,
            LocalBoundaryQuerier = this,
            AnnotationService = _annotationServerice,
        };

        return _entityManager.CreateEntity(Params);
    }

    public void RemoveCrowdEntity(UniqueId InEntityId)
    {
        _entityManager.DeleteEntity(InEntityId);
    }

    public UniqueId HitCrowdEntity(double x, double y)
    {
        var hitPos = FixMath.F64Vec3.FromDouble(x, MapHeight.Double, y);
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
        var Position = FixMath.F64Vec3.FromDouble(x, MapHeight.Double, y);
        //_entityManager.ForEachEntity((InEntity) => { 
        //    if (null != InEntity)
        //    {
        //        InEntity.TargetLocation = Position;
        //    }
        //});
        for (var i = 0; i < _selectEntities.Count; ++i)
        {
            var entity = _entityManager.GetEntityById(_selectEntities[i]) as MovableEntity;
            if (null != entity)
            {
                entity.TargetLocation = Position;
            }
        }
    }

    public void DrawCrowdEntity()
    {
        _entityManager.ForEachEntity((InEntity) =>
        {
            if (InEntity is MovableEntity entity)
            {
                if (Debug.IsSimulationMode(eSimulationMode.Normal))
                {
                    var index = SelectEntities.FindIndex(Id => Id == entity.ID);
                    entity.OnDraw(index != -1);
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

    public void Layout()
    {
        var propertyChanged = false;

        if (ImGui.Button("Dump entity position"))
        {
            _entityManager.ForEachEntity((InEntity) =>
            {
                if (InEntity is MovableEntity entity)
                {
                    TestDaedalusSampleTool.Logger.Information($"id:{InEntity.ID.Id}, pos:{entity.Position}");
                }
            });
        }

        if (ImGui.Button("Create test entities"))
        {
            var testPositions = new Vector2[] 
            {
                
            };

            for (var i = 0; i < testPositions.Length; ++i)
            {
                var pos = testPositions[i];
                AddCrowdEntity(pos.x, pos.y);
            }
        }

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
        var currTemplate = MovableEntityTemplates[_templateIndex];

        // debug toggles
        for (var i = 0; i < currTemplate.DebugVec3Toggles.Length; ++i)
        {
            var type = (eDebugVec3Item)i;
            propertyChanged |= ImGui.Checkbox(type.ToString(), ref currTemplate.DebugVec3Toggles[i]);
        }

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

        var src = start;
        var dst = start + direction * 100.0f;
        var bmin = new RcVec3f(0.0f, _draw.MapHeight, 0.0f);
        var bmax = new RcVec3f(_tool.Mesh._width.Float, _draw.MapHeight, _tool.Mesh._height.Float);
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
        if (null == _tool.Mesh) return;

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

                //_tool.EntityAI.x = _tool.StartPoint.Value.x;
                //_tool.EntityAI.y = _tool.StartPoint.Value.z;
                //_tool.EntityAI._x = _tool.EntityAI.x;
                //_tool.EntityAI._y = _tool.EntityAI.y;

                //_tool.Pathfinder.findPath(_tool.EndPoint.Value.x, _tool.EndPoint.Value.z, _tool.Path);

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

        var geom = _sample.GetInputGeom();
        var bound_min = geom.GetMeshBoundsMin();
        var bound_max = geom.GetMeshBoundsMax();

        if (_draw == null)
        {
            _draw = new DrawInterfaceImplement(dd);
            _draw.MapHeight = bound_max.Y + 0.5f;

            _view = new SimpleView(_draw);
            _tool.MapHeight = FixMath.F64.FromDouble(bound_max.Y + 0.5f);
            _tool.DrawInterface = _draw;

            _physicsWorldDrawer = new PhysicsWorldGizmosDrawer(_draw);
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
        {
            _tool.EntityManager.ForEachEntity((InEntity) =>
            {
                if (null != InEntity && null != InEntity.PhysicsBody)
                {
                    Volatile.Color edgeColor        = Volatile.Color.white;
                    Volatile.Color normalColor      = Volatile.Color.green;
                    Volatile.Color bodyOriginColor  = Volatile.Color.blue;
                    Volatile.Color shapeOriginColor = Volatile.Color.yellow;
                    Volatile.Color bodyAabbColor    = Volatile.Color.magenta;
                    Volatile.Color shapeAabbColor   = Volatile.Color.cyan;
                    InEntity.PhysicsBody.GizmoDraw(_physicsWorldDrawer, edgeColor, normalColor, bodyOriginColor, shapeOriginColor, bodyAabbColor, shapeAabbColor, FixMath.NET.Fix64.One);
                }
            });
        }

        // draw world axes
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
        ImGui.NewLine();
        if (Debug.IsSimulationMode(eSimulationMode.Normal))
        {
            // 正常模拟模式
            _tool.Layout();

            if (ImGui.Button("Playback Mode"))
            {
                Debug.SetSimlationMode(eSimulationMode.Playback);
            }

            ImGui.NewLine();
            
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
        else if (Debug.IsSimulationMode(eSimulationMode.Playback))
        {
            // 回放模式
            if (ImGui.Button("Normal Mode"))
            {
                Debug.SetSimlationMode(eSimulationMode.Normal);
            }

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

        RandomHelpers.InitialRandom((int)DateTime.Now.Ticks);

        if (null != _tool) _tool.Destroy();

        _tool = new TestFixedCrowdTool();
        if (null != _tool) _tool.Start();

        _tool.BuildGraphMesh(FixMath.F64.FromDouble(mapWidth), FixMath.F64.FromDouble((int)mapHeight));

        Logger.Information($"init graph mesh, mapWidth:{mapWidth}, mapHeight:{mapHeight}");
    }
}
