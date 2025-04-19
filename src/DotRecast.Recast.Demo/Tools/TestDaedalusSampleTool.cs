using System;
using System.Collections.Generic;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Recast.Demo.Draw;
using DotRecast.Recast.Toolset;
using ImGuiNET;
using Serilog;
using static DotRecast.Recast.Demo.Draw.DebugDraw;
using hxDaedalus.factories;
using UnityEngine;
using DotRecast.Core.Collections;
using Pathfinding.Crowds;
using Pathfinding.Util;
using hxDaedalus.ai;
using SharpSteer2.Helpers;

namespace DotRecast.Recast.Demo.Tools;

public class TestDaedalusToolMode
{
    public static readonly TestDaedalusToolMode ADD_OBSTACLE = new TestDaedalusToolMode(0, "Add Obstacle");
    public static readonly TestDaedalusToolMode PATH_FINDER = new TestDaedalusToolMode(1, "Path Finder");
    public static readonly TestDaedalusToolMode ADD_CROWD_ENTITY = new TestDaedalusToolMode(2, "Add Crowd Entity");
    public static readonly TestDaedalusToolMode SELECT_CROWD_ENTITY = new TestDaedalusToolMode(3, "Select Crowd Entity");
    public static readonly TestDaedalusToolMode MOVE_CROWD_ENTITY = new TestDaedalusToolMode(4, "Move Crowd Entity");
    public static readonly TestDaedalusToolMode SELECT_MESH_FACE = new TestDaedalusToolMode(5, "Select Mesh Face");

    public static readonly RcImmutableArray<TestDaedalusToolMode> Values = RcImmutableArray.Create(
            ADD_OBSTACLE,
            PATH_FINDER,
            ADD_CROWD_ENTITY,
            SELECT_CROWD_ENTITY,
            MOVE_CROWD_ENTITY,
            SELECT_MESH_FACE
        );

    public readonly int Idx;
    public readonly string Label;

    private TestDaedalusToolMode(int idx, string label)
    {
        Idx = idx;
        Label = label;
    }
}

public class DrawInterfaceImplement : IDrawInterface
{
    private DebugDraw m_draw = null;

    public float TerrainHeight { get; set; }

    public DrawInterfaceImplement(DebugDraw draw)
    {
        m_draw = draw;
        Debug.drawInterface = this;
    }

    public Vector3 ToVec3(FixMath.F64Vec2 v)
    {
        return new Vector3(v.X.Float, TerrainHeight, v.Y.Float);
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

    public void DrawArrow(Vector3 start, Vector3 end, Vector2 arrowSize, float lineWidth, Color c)
    {
        if (m_draw != null)
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            m_draw.DebugDrawArrow(start.x, start.y, start.z, end.x, end.y, end.z, arrowSize.x, arrowSize.y, color, lineWidth);
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

    public void DrawSolidPlane(Vector3 point, Vector3 normal, Vector2 size, UnityEngine.Color c)
    {
        if (m_draw != null)
        {
            // (x - p) dot n = 0
            var n = FixMath.F64Vec3.FromFloat(normal.x, normal.y, normal.z);
            n.FindBestAxisVectors(out var axis1, out var axis2);

            var u = axis1.Cast() * size.x * 0.5f;
            var v = axis2.Cast() * size.y * 0.5f;
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            var points = new Vector3[4] {
                point + u + v,
                point - u + v,
                point - u - v,
                point + u - v
            };

            m_draw.Begin(DebugDrawPrimitives.QUADS);
            // 正面
            for (int i = 0; i < 4; i++)
            {
                ref var p = ref points[i];
                m_draw.Vertex(new float[] { p.x, p.y, p.z }, color);
            }
            // 反面
            for (int i = 3; i >= 0; i--)
            {
                ref var p = ref points[i];
                m_draw.Vertex(new float[] { p.x, p.y, p.z }, color);
            }
            m_draw.End();
        }
    }

    public void DrawSolidCube(Vector3 p, Quaternion q, Vector3 size, Color c)
    {
        if (m_draw != null)
        {
            int color = DuRGBA((int)(c.r * 255), (int)(c.g * 255), (int)(c.b * 255), (int)(c.a * 255));

            int[] fcol = new int[6];
            DuCalcBoxColors(fcol, color, color);

            var halfSize = size * 0.5f;
            var min = p - halfSize;
            var max = p + halfSize;
            m_draw.DebugDrawBox(min.x, min.y, min.z, max.x, max.y, max.z, fcol);
        }
    }
}

// 问题记录：Unity使用左手坐标系，这里DebugDraw使用的是右手坐标系，会导致
// VO逻辑和渲染出来的left和right边是反着的（忽略该问题）
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
    public IDrawInterface DrawInterface 
    { 
        set 
        {
            _annotationServerice = new EntityAnnotationServerice(value);
        } 
    }

    public double RandomRange(double min, double max)
    {
        return min + rand.NextDouble() * (max - min);
    }

    public string GetName()
    {
        return "Test Daedalus Tool";
    }

    public void Start(double mapWidth, double mapHeight)
    {
        if (null != _entityManager)
        {
            var mapYExtent = FixMath.F64.FromDouble(5.0);
            var param = new IMovableEntityManager.FInitializeParams()
            {
                MapBoundsMin = new FixMath.F64Vec3(FixMath.F64.Zero, FixMath.F64.Zero - mapYExtent / 2, FixMath.F64.Zero),
                MapBoundsMax = new FixMath.F64Vec3(FixMath.F64.FromDouble(mapWidth), FixMath.F64.Zero + mapYExtent / 2, FixMath.F64.FromDouble(mapHeight)),
                MapCellDivs = 10,
            };
            _entityManager.Initialize(param);
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
            _entityManager.TickReplay(FixMath.F64.FromDouble(inDeltaTime));
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
        for (int i=0; i<0; ++i)
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
                var v = FixMath.F64Vec3.FromFloat((float)resultPath[i], MapHeight, (float)resultPath[i+1]);
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

        System.Func<hxDaedalus.data.Face, bool> CheckIntersectionFaceAndCircle = (face) => {
            // 这里改成Face的AABB盒是否和Circle相交判定，比只判定3个顶点更准确一些
            global::hxDaedalus.iterators.FromFaceToInnerEdges iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
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

                var pos = VertextToF64Vec3(innerEdge.get_originVertex());
                min.X = FixMath.F64.Min(min.X, pos.X);
                min.Y = FixMath.F64.Min(min.Y, pos.Y);
                min.Z = FixMath.F64.Min(min.Z, pos.Z);

                max.X = FixMath.F64.Max(max.X, pos.X);
                max.Y = FixMath.F64.Max(max.Y, pos.Y);
                max.Z = FixMath.F64.Max(max.Z, pos.Z);
            }

            if (min.X <= max.X && min.Y <= max.Y && min.Z <= max.Z)
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
    public UniqueId AddCrowdEntity(double x, double y)
    {
        var template = MovableEntityTemplates[_templateIndex];
        var tid = UniqueId.FromName($"Movable r:{template.Radius} hash:{template.GetHashCode()}");
        if (null != _entityManager.FindTemplate(tid))
        {
            _entityManager.RegisterTemplate(tid, template);
        }

        var Params = new CreateEntityParams() 
        {
            EntityId = UniqueId.InvalidID,
            SpawnPosition = FixMath.F64Vec3.FromDouble(x, MapHeight, y),
            SpawnRotation = FixMath.F64Quat.Identity,

            TemplateId = tid,
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
        var hitPos = FixMath.F64Vec3.FromDouble(x, MapHeight, y);
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
        var Position = FixMath.F64Vec3.FromDouble(x, MapHeight, y);
        for (var i = 0; i < _selectEntities.Count; ++i)
        {
            _entityManager.MoveEntity(_selectEntities[i], Position);
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

    public void DrawFace(hxDaedalus.data.Face face, hxDaedalus.view.SimpleView view)
    {
        for (var i = 0; i < Mesh._faces.length; ++i)
        {
            var f = (hxDaedalus.data.Face)Mesh._faces[i];
            if (f == face)
            {
                global::hxDaedalus.iterators.FromFaceToInnerEdges iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
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

    public void DrawEdge(hxDaedalus.data.Edge edge)
    {
        
    }

    public void DebugDraw(hxDaedalus.view.SimpleView view)
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
        for (var i = 0; i < TMovableEntityTemplate.DebugVec3Toggles.Length; ++i)
        {
            var type = (eDebugVec3Item)i;
            ImGui.Checkbox(type.ToString(), ref TMovableEntityTemplate.DebugVec3Toggles[i]);
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

public class TestDaedalusSampleTool : ISampleTool
{
    public static readonly ILogger Logger = Log.ForContext<TestNavmeshSampleTool>();

    private DemoSample _sample;

    private TestDaedalusTool _tool = null;

    private DrawInterfaceImplement _draw = null;

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

    bool GetRaycastHitPos(RcVec3f start, RcVec3f direction, out RcVec3f outHitPos)
    {
        // 平面 y = _draw.MapHeight
        outHitPos = new RcVec3f();

        var src = start;
        var dst = start + direction * 100.0f;
        var bmin = new RcVec3f(0.0f, _draw.TerrainHeight, 0.0f);
        var bmax = new RcVec3f((float)_tool.Mesh._width, _draw.TerrainHeight, (float)_tool.Mesh._height);
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
            _draw.TerrainHeight = bound_max.Y + 0.5f;

            _view = new hxDaedalus.view.SimpleView(_draw);
            _tool.MapHeight = bound_max.Y + 0.5f;
            _tool.DrawInterface = _draw;
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
                var v0 = new UnityEngine.Vector3((float)_tool.Path[0], _draw.TerrainHeight, (float)_tool.Path[1]);
                var PointSize = UnityEngine.Vector3.one * 0.2f;
                _draw.DrawCube(v0, PointSize, UnityEngine.Color.red);
                for (var i = 2; i < _tool.Path.length; i += 2)
                {
                    var v1 = new UnityEngine.Vector3((float)_tool.Path[i], _draw.TerrainHeight, (float)_tool.Path[i + 1]);
                    _draw.DrawLine(v0, v1, UnityEngine.Color.green);
                    _draw.DrawCube(v1, PointSize, UnityEngine.Color.red);
                    v0 = v1;
                }
            }

            // draw face
            _tool.DrawFace(_tool.HitFace, _view);
        }
        else if (m_mode == TestDaedalusToolMode.SELECT_CROWD_ENTITY)
        {
            if (_tool.IsSelecting)
            {
                var p = (_tool.SelectionStartPosition + _tool.SelectionEndPosition) * 0.5f;
                var size = _tool.SelectionEndPosition - _tool.SelectionStartPosition;
                _draw.DrawSolidCube(new Vector3(p.X, p.Y, p.Z), Quaternion.identity, new Vector3(Math.Abs(size.X), 0.01f, Math.Abs(size.Z)), new Color(1.0f, 0.0f, 0.0f, 0.3f));
                // _draw.DrawCircle(new Vector3(p.X, p.Y, p.Z), size.X)
            }
        }

        // draw crowd entities
        _tool.DrawCrowdEntity();

        // debug draw
        _tool.DebugDraw(_view);

        // draw world axes
        {
            var o = Vector3.zero; o.y += 3.0f;
            var x = o + Vector3.right * 5;
            var y = o + Vector3.up * 5;
            var z = o + Vector3.forward * 5;
            _draw.DrawArrow(o, x, new Vector2(0.0f, 0.2f), 1.0f, Color.red);
            _draw.DrawArrow(o, y, new Vector2(0.0f, 0.2f), 1.0f, Color.green);
            _draw.DrawArrow(o, z, new Vector2(0.0f, 0.2f), 1.0f, Color.blue);
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

        _tool = new TestDaedalusTool();
        if (null != _tool) _tool.Start(mapWidth, mapHeight);

        _tool.BuildGraphMesh(0, 0, 300, 150);

        Logger.Information($"init graph mesh, mapWidth:{mapWidth}, mapHeight:{mapHeight}");
    }
}
