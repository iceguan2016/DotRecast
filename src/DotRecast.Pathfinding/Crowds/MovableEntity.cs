using System;
using SharpSteer2;
using SharpSteer2.Database;
using DotRecast.Pathfinding.Util;
using System.Collections.Generic;
using SharpSteer2.Pathway;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using Game.Utils;

namespace DotRecast.Pathfinding.Crowds
{
    public class TemplateMovableEntity
    {
        // radius
        public FixMath.F64 Radius = FixMath.F64.FromFloat(0.5f);
        // maximum move speed
        public FixMath.F64 MaxSpeed = FixMath.F64.FromFloat(6.0f);
        // maximum force
        public FixMath.F64 MaxForce = FixMath.F64.FromFloat(27.0f);
        // query local boundary radius
        public FixMath.F64 QueryLocalBoundaryRadius = FixMath.F64.FromFloat(5.0f);
        // query local neighbors radius
        public FixMath.F64 QueryLocalNeighborRadius = FixMath.F64.FromFloat(5.0f);
        //
        public FixMath.F64 FollowPathAheadTime = FixMath.F64.FromFloat(3.0f);
        public FixMath.F64 FollowPathWeight = FixMath.F64.FromFloat(1.0f);

        public FixMath.F64 AvoidObstacleAheadTime = FixMath.F64.FromFloat(0.1f);
        public FixMath.F64 AvoidObstacleWeight = FixMath.F64.FromFloat(1.0f);

        public FixMath.F64 AvoidNeighborAheadTime = FixMath.F64.FromFloat(1.0f);
        public FixMath.F64 AvoidNeighborWeight = FixMath.F64.FromFloat(1.0f);
    }

    public class MovableEntity : SimpleVehicle
    {
        // 单位状态
        public enum  eEntityState
        {
            Idle = 0,
	        Moving,
        };

        // a pointer to this boid's interface object for the proximity database
        private ITokenForProximityDatabase<IVehicle> _proximityToken;

        // Config values
        public override FixMath.F64 MaxForce { get { return Template.MaxForce; } }
        public override FixMath.F64 MaxSpeed { get { return Template.MaxSpeed; } }

        public TemplateMovableEntity Template { get; set; }

        public IMovableEntityManager EntityManager { get; private set; }

        public ILocalBoundaryQuerier LocalBoundaryQuerier { get; private set; }

        public IPathwayQuerier PathwayQuerier { get; private set; }
        
        // Entity unique id
        public UniqueId ID { get; set; }

        // Move destination
        protected bool isTargetDirty { get; private set; }
        protected FixMath.F64Vec3? targetLocation = null;
        public FixMath.F64Vec3? TargetLocation { 
            get
            {
                return targetLocation;
            }
            
            set 
            {
                isTargetDirty = null == targetLocation || FixMath.F64Vec3.DistanceFast(targetLocation.Value, value.Value) > FixMath.F64.Two;
                if (isTargetDirty) 
                {
                    targetLocation = value;
                    SetEntityState(null != targetLocation? eEntityState.Moving : eEntityState.Idle);
                }
            }
        }

        public MovableEntityDebuger Debuger { get; private set; }
        public IAnnotationService Annotation { get { return annotation; } }

        // route for path following (waypoints and legs)
        private PolylinePathway Pathway = null;
        // True means walking forward along the path, false means walking backward along the path
        private bool _pathDirection = true;
        private FixMath.F64Vec3? _pathReferencePosition = null;
        // Record the most recent avoidance direction
        private SteerLibrary.AvoidReferenceInfo _avoidReferenceInfo = new AvoidReferenceInfo();
        private IVehicle.FAvoidNeighborInfo _avoidNeighborInfo = new IVehicle.FAvoidNeighborInfo();
        public IVehicle.FAvoidNeighborInfo AvoidNeighborInfo { get { return _avoidNeighborInfo; } }

        // local boundary
        public static readonly int  MaxBoundarySegmentNum = 10;
        private BoundarySegement[]  _boundarySegements = null;
        private int                 _boundarySegmentNum = 0;
        private List<IObstacle>     _boundaryObstacles = new List<IObstacle>();

        // local neighbors
        private List<IVehicle>      _neighbors = new List<IVehicle>(); 

        // state bits
        private uint                _stateBitsValue = 0;

        public void SetEntityState(eEntityState inState) { _stateBitsValue |= (uint)(1 << (int)inState); }
        public void ClearEntityState(eEntityState inState) { _stateBitsValue &= ~(uint)(1 << (int)inState); }
        public bool HasEntityState(eEntityState inState) { return (_stateBitsValue & (uint)(1 << (int)inState)) > 0; }

        // constructor
        public MovableEntity(IMovableEntityManager manager, 
                             IPathwayQuerier pathwayQuerier = null, 
                             ILocalBoundaryQuerier bounaryQuerier = null, 
                             IAnnotationService annotations = null)
            : base(annotations)
        {
            EntityManager = manager;
            PathwayQuerier = pathwayQuerier;
            LocalBoundaryQuerier = bounaryQuerier;
            if (null != annotation) annotation.IsEnabled = true;
        }

        // reset state
        public override void Reset()
        {
            // reset the vehicle
            base.Reset();

            // randomize initial orientation
            //RegenerateOrthonormalBasisUF(Vector3Helpers.RandomUnitVector());
            //Vector3 d = Vector3Helpers.RandomUnitVector();
            //d.X = Math.Abs(d.X);
            //d.Y = 0;
            //d.Z = Math.Abs(d.Z);
            //RegenerateOrthonormalBasisUF(d);

            //// randomize initial position
            //Position = Vector3.UnitX * 10 + (Vector3Helpers.RandomVectorInUnitRadiusSphere() * 20);

            // notify proximity database that our position has changed
            //FIXME: SimpleVehicle::SimpleVehicle() calls reset() before proximityToken is set
            
        }

        public void NewPD(IProximityDatabase<IVehicle> pd)
        {
            // delete this boid's token in the old proximity database
            if (_proximityToken != null)
            {
                _proximityToken.Dispose();
                _proximityToken = null;
            }

            // allocate a token for this boid in the proximity database
            if(null != pd) _proximityToken = pd.AllocateToken(this);
        }

        public List<IVehicle> FindNeighbors(FixMath.F64 inRadius)
        {
            if (null == _proximityToken) return null;
            
            var result = new List<IVehicle>();
            _proximityToken.FindNeighbors(Position, inRadius, result);
            return result;
        }

        public virtual void OnCreate()
        {
            _boundarySegements = new BoundarySegement[MaxBoundarySegmentNum];
            Array.Fill(_boundarySegements, default);
            _boundarySegmentNum = 0;

            // initial slow speed
            Speed = (MaxSpeed * FixMath.F64.FromFloat(0.3f));
            Radius = Template.Radius;

            if (_proximityToken != null)
                _proximityToken.UpdateForNewPosition(Position);

#if ENABLE_STEER_AGENT_DEBUG
            Debuger = new MovableEntityDebuger();
#endif
        }

        public virtual void OnDelete()
        {
            Debuger = null;
        }

        public virtual void OnUpdate(FixMath.F64 inDeltaTime)
        {
            if (!HasEntityState(eEntityState.Moving)) return;
            if (null == targetLocation) return;

            // update pathway, check destination still valid?
            if (isTargetDirty && null != PathwayQuerier && null != TargetLocation)
            {
                isTargetDirty = false;
                Pathway = PathwayQuerier.FindPath(this, targetLocation.Value);
            }

            // update neighbors
            if (null != _proximityToken)
            {
                _neighbors.Clear();
                _proximityToken.FindNeighbors(Position, Template.QueryLocalNeighborRadius, _neighbors);
            }

            // update local boundary
            if (null != LocalBoundaryQuerier)
            {
                _boundarySegmentNum = LocalBoundaryQuerier.QueryBoundaryInCircle(this, Template.QueryLocalBoundaryRadius, _boundarySegements);
            }

            // determine steering force
#if ENABLE_STEER_AGENT_DEBUG
            ref var info = ref Debuger.EntityInfoBuff.Alloc(EntityManager.FrameNo);
            info.maxSpeed = MaxSpeed;
            info.radius = Radius;
            info.position = Position;
            info.velocity = Velocity;
            info.forward = Forward;
            info.side = Side;
            info.up = Up;
            info.steerPosition = _pathReferencePosition?? TargetLocation.Value;
#endif

            var steerForce = determineCombinedSteering(inDeltaTime);
            if (steerForce != FixMath.F64Vec3.Zero)
            {
                ApplySteeringForce(steerForce, inDeltaTime);

                if (null != _proximityToken)
                    _proximityToken.UpdateForNewPosition(Position);
            }
        }

        public virtual void OnDraw()
        {
            // draw vehicle
            Draw.drawBasic2dCircularVehicle(annotation, this, Colors.Gray50);
            // drawTrail();
            // draw transform
            Draw.drawAxes(annotation, this, FixMath.F64Vec3.FromFloat(1f, 1f, 1f));
            // draw pathway
            if (null != Pathway)
            {
                var size = FixMath.F64.FromFloat(0.2f);
                FixMath.F64Vec3? last = null;
                foreach(var p in Pathway.Points)
                {
                    annotation.CircleXZ(size, p, Colors.Red, 10);
                    if (null != last)
                    {
                        annotation.Line(last.Value, p, Colors.Green, FixMath.F64.One);
                    }
                    last = p;
                }
            }
            // draw local boundary
            Draw.drawCircleOrDisk(annotation, Template.QueryLocalBoundaryRadius, FixMath.F64Vec3.Up, Position, Colors.Yellow, 10, false, false);

            if (_boundaryObstacles.Count > 0)
            {
                for (var i = 0; i < _boundaryObstacles.Count; ++i)
                {
                    _boundaryObstacles[i].draw(annotation, false, Colors.Yellow, Position);
                }
            }

            // draw obstacle avoid region
            var testPoint = PredictFuturePosition(Template.AvoidObstacleAheadTime);
            // var point = (Position + testPoint) * FixMath.F64.Half;
            // point.Y += FixMath.F64.Half;
            //var length = testPoint.Distance(Position);
            //var rotation = this.ToMatrix().ToQuat();
            //Draw.drawBoxOutline(annotation, point, rotation, new FixMath.F64Vec3(FixMath.F64.FromFloat(1.0f), FixMath.F64.Zero, length), Colors.Green, FixMath.F64.FromFloat(1.0f));
            Draw.drawLineAlpha(annotation, Position, testPoint, Colors.Yellow, FixMath.F64.FromFloat(0.4f));
        }

        // compute combined steering force: move forward, avoid obstacles
        // or neighbors if needed, otherwise follow the path and wander
        FixMath.F64Vec3 determineCombinedSteering(FixMath.F64 elapsedTime)
        {
            if (null == EntityManager || null == targetLocation)
                return FixMath.F64Vec3.Zero;

            var forceScale = FixMath.F64.FromFloat(1.0f);
#if ENABLE_STEER_AGENT_DEBUG
	        ref var info = ref Debuger.SteerFoceInfoBuff.Alloc(EntityManager.FrameNo);
	        info.reset();
	        info.position = Position;
	        info.maxForce = MaxForce;
#endif

            // probability that a lower priority behavior will be given a
            // chance to "drive" even if a higher priority behavior might
            // otherwise be triggered.
            var leakThrough = FixMath.F64.FromFloat(0.1f);

            // steering to seek target
            var steeringForce = FixMath.F64Vec3.Zero;
            FixMath.F64Vec3? referencePoint = null;
            // path follow corner point
            if (null != Pathway)
            {
                var predictionTime = Template.FollowPathAheadTime;
                steeringForce = this.SteerToFollowPath(_pathDirection, predictionTime, Pathway, MaxSpeed, out var currentPathDistance, annotation) * Template.FollowPathWeight;

                // float pathDistanceOffset = (_pathDirection ? 1 : -1) * predictionTime * Speed;
                // referencePoint = Pathway.MapPathDistanceToPoint(currentPathDistance + FixMath.F64.FromFloat(0.5f));
            }
            else
            {
                steeringForce = SteerForSeek(targetLocation.Value) * Template.FollowPathWeight;
            }
            steeringForce = steeringForce.TruncateLength(MaxForce);
            _pathReferencePosition = referencePoint;

#if ENABLE_STEER_AGENT_DEBUG
            info.targetForce = steeringForce * forceScale;
#endif

            // avoid obstacles
            _boundaryObstacles.Clear();
            // var obstacles = new List<IObstacle>();
            var obstacleAvoidance = FixMath.F64Vec3.Zero;
            if (leakThrough < RandomHelpers.Random())
            {
                for (var i = 0; i < _boundarySegmentNum; ++i)
                {
                    var boundary = _boundarySegements[i];
                    var v = boundary.End - boundary.Start;
                    v.Y = FixMath.F64.Zero;
                    var s = v.Normalize();
                    var f = Vector3Helpers.Up.Cross(s);
                    var u = f.Cross(s);
                    var p = (boundary.End + boundary.Start) * FixMath.F64.Half;

                    var w = v.Length2D();
                    var h = FixMath.F64.FromFloat(1.0f);
                    var obstacle = new RectangleObstacle(w, h, s, u, f, p, seenFromState.outside);
                    _boundaryObstacles.Add(obstacle);

                }

                if (_boundaryObstacles.Count > 0)
                {
                   obstacleAvoidance = SteerToAvoidObstacles(Template.AvoidObstacleAheadTime, _boundaryObstacles, ref _avoidReferenceInfo) * Template.AvoidObstacleWeight;

#if ENABLE_STEER_AGENT_DEBUG
			        info.obstacleForce = obstacleAvoidance * forceScale;
#endif
                }
            }

            // if obstacle avoidance is needed, do it
            if (obstacleAvoidance != FixMath.F64Vec3.Zero)
            {
                steeringForce += obstacleAvoidance;
            }
            else
            {
                // otherwise consider avoiding collisions with others
                List<IVehicle> neighbors = new List<IVehicle>();
                var collisionAvoidance = FixMath.F64Vec3.Zero;
                if (leakThrough < RandomHelpers.Random())
                {
                    for (int i = 0; i < _neighbors.Count; ++i)
                    {
                        var neighbor = _neighbors[i] as MovableEntity;
                        if (null == neighbor || ID == neighbor.ID) continue;
                        if (neighbor.HasEntityState(eEntityState.Moving)) continue;
                        neighbors.Add(neighbor);
                    }

                    // collisionAvoidance = steerToAvoidNeighbors(timeCollisionWithNeighbor, neighbors);
                    collisionAvoidance = SteerToAvoidNeighbors(Template.AvoidNeighborAheadTime, neighbors, ref _avoidNeighborInfo);
                    collisionAvoidance = collisionAvoidance.Normalize() * MaxForce * Template.AvoidNeighborWeight;

#if ENABLE_STEER_AGENT_DEBUG
			        info.avoidNeighborFoce = collisionAvoidance * forceScale;
#endif
                }

                // if collision avoidance is needed, do it
                if (collisionAvoidance != FixMath.F64Vec3.Zero)
                {
                    steeringForce += collisionAvoidance;
                }
                else
                {
                    // do (interactively) selected type of path following
                    //const float pfLeadTime = 3;
                    //const Vec3 pathFollow =
                    //    (gUseDirectedPathFollowing ?
                    //        steerToFollowPath(pathDirection, pfLeadTime, *path) :
                    //        steerToStayOnPath(pfLeadTime, *path));

                    //// add in to steeringForce
                    //steeringForce += pathFollow * 0.5;
                }
            }

            var totalForce = steeringForce.SetYtoZero() * forceScale;
#if ENABLE_STEER_AGENT_DEBUG
	        info.steerForce = totalForce;
#endif

            return totalForce;
        }

        public override FixMath.F64Vec3 GetAvoidObstacleDirection(ref PathIntersection pathIntersection) 
        { 

            return FixMath.F64Vec3.Zero; 
        }

        public override FixMath.F64Vec3 GetAvoidNeighborDirection(IVehicle threat, PathIntersection? intersection, ref IVehicle.FAvoidNeighborInfo info) 
        {
            // 选择最优避让方向规则：
            // (1) 与desiredVelocity方向最接近
            // (2) 和上一次选择side保持一致
            // (3) 被避让单位是否在避让自己，选择与其保持一致的避让方向（比如都选左，或者右）
            // (4) 检查选择的避让方向是否会撞墙
            if (threat is MovableEntity entity)
            {
                var relativePosition = threat.Position - Position;
                var distSq = relativePosition.LengthSquared2D();
                var combinedRadius = threat.Radius + Radius;
                var combinedRadiusSq = combinedRadius * combinedRadius;

                FixMath.F64Vec2 u = FixMath.F64Vec2.Zero;
                FixMath.F64Vec2 left, right;

                var leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                left = new FixMath.F64Vec2(relativePosition.X * leg - relativePosition.Y * combinedRadius,
                        relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                right = -new FixMath.F64Vec2(relativePosition.X * leg + relativePosition.Y * combinedRadius,
                        -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;

                var side = IVehicle.eAvoidVOSide.Left;
                if (info.VOSide == IVehicle.eAvoidVOSide.None)
                {
                    
                    // 选择和对方一致的避让方向
                    if (entity.AvoidNeighborInfo.EntityId == ID)
                    {
                        side = entity.AvoidNeighborInfo.VOSide;
                    }
                    else
                    {
                        // 选择不撞墙的方向
                        var minDistanceToCollision = Template.AvoidObstacleAheadTime * Speed;
                        var testDirs = new FixMath.F64Vec2[] { left, right };
                        var testSides = new IVehicle.eAvoidVOSide[] { IVehicle.eAvoidVOSide.Left, IVehicle.eAvoidVOSide.Right };

                        var movable = new SimpleVehicle();
                        movable.Position = Position;
                        movable.Radius = Radius;

                        for (var dir = 0; dir < testDirs.Length; ++dir)
                        {
                            var forward = testDirs[dir].Cast(FixMath.F64.Zero);
                            movable.Forward = forward;
                            movable.Side = FixMath.F64Vec3.Cross(forward, FixMath.F64Vec3.Up);
                            movable.Up = FixMath.F64Vec3.Up;

                            var findDir = true;
                            PathIntersection pathIntersection = new PathIntersection();
                            for (var i = 0; i < _boundaryObstacles.Count; ++i)
                            {
                                var boundary = _boundaryObstacles[i];
                                if (null == boundary) continue;
                                boundary.findIntersectionWithVehiclePath(movable, ref pathIntersection);
                                if (pathIntersection.intersect && pathIntersection.distance < minDistanceToCollision)
                                {
                                    // 会碰撞Wall，此方向无效
                                    findDir = false;
                                    break;
                                }
                            }

                            if (findDir) 
                            {
                                side = testSides[dir];
                                break;
                            }
                        }
                    }
                }
                else
                {
                    side = info.VOSide;
                }

                // 根据side返回避让方向
                info.VOSide = side;
                info.EntityId = entity.ID;
                return side == IVehicle.eAvoidVOSide.Left? left.Cast(FixMath.F64.Zero) : right.Cast(FixMath.F64.Zero);
            }
            return FixMath.F64Vec3.Zero; 
        }

        public override void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision)
        {

        }

        public override void AnnotationAvoidObstacle(FixMath.F64 minDistanceToCollision, SharpSteer2.Obstacles.PathIntersection nearest) 
        {
            if (nearest.intersect)
            {
#if ENABLE_STEER_AGENT_DEBUG
                ref var pathIntersectionInfo = ref Debuger.PathInterSectionBuff.Alloc(EntityManager.FrameNo);
                pathIntersectionInfo = nearest;

                if (nearest.obstacle is RectangleObstacle obstacle)
                {
                    ref var rectangeObstacleInfo = ref Debuger.HitRectangeObstacleBuff.Alloc(EntityManager.FrameNo);
                    rectangeObstacleInfo.Position = obstacle.Position;
                    rectangeObstacleInfo.Forward = obstacle.Forward;
                    rectangeObstacleInfo.Side = obstacle.Side;
                    rectangeObstacleInfo.Up = obstacle.Up;
                    rectangeObstacleInfo.Width = obstacle.width;
                    rectangeObstacleInfo.Height = obstacle.height;
                }
#endif
            }
        }

        public override void AnnotationAvoidCloseNeighbor(IVehicle other, FixMath.F64 additionalDistance) 
        {
        }

        public override void AnnotationAvoidCloseNeighbor(IVehicle threat, FixMath.F64Vec3 avoidDirection, IVehicle.FAvoidNeighborInfo info)
        {
#if ENABLE_STEER_AGENT_DEBUG
            ref var avoidCloseNeighorInfo = ref Debuger.SteerAvoidCloseNeighborInfoBuff.Alloc(EntityManager.FrameNo);
            avoidCloseNeighorInfo.threatPosition = threat.Position;
            avoidCloseNeighorInfo.threatRadius = threat.Radius;
            avoidCloseNeighorInfo.avoidDirection = avoidDirection;
            avoidCloseNeighorInfo.avoidNeighborInfo = info;
#endif
        }

        public override void AnnotationAvoidNeighbor(IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture) 
        {
        }

        public override void AnnotationAvoidNeighbor(IVehicle threat, SharpSteer2.Obstacles.PathIntersection intersection) 
        {
        }
    }
}
