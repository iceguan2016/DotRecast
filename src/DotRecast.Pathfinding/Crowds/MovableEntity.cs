using System;
using SharpSteer2;
using SharpSteer2.Database;
using DotRecast.Pathfinding.Util;
using System.Collections.Generic;
using SharpSteer2.Pathway;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using Game.Utils;
using System.Threading;
using Pathfinding.Crowds;
using System.Diagnostics.Contracts;

namespace DotRecast.Pathfinding.Crowds
{
    public enum eDebugVec3Item
    { 
        Velocity = 0,
        ForwardMoveForce,
        PathFollowForce,
        AvoidNeighborForce,
        AvoidObstacleForce,
        FlockSeparationForce,
        FlockAligmentForce,
        FlockCohesionForce,
        TotalSteerForce,

        Count,
    }
    public class TemplateMovableEntity
    {
        // radius
        public FixMath.F64 Radius = FixMath.F64.FromFloat(0.5f);
        // maximum move speed
        public FixMath.F64 MaxSpeed = FixMath.F64.FromFloat(6.0f);
        // maximum force
        public FixMath.F64 MaxForce = FixMath.F64.FromFloat(27.0f);

        //
        public FixMath.F64 ForwardMoveWeight = FixMath.F64.FromFloat(1.0f);

        public FixMath.F64 FollowPathAheadTime = FixMath.F64.FromFloat(3.0f);
        public FixMath.F64 FollowPathWeight = FixMath.F64.FromFloat(1.0f);

        public FixMath.F64 AvoidObstacleAheadTime = FixMath.F64.FromFloat(0.1f);
        public FixMath.F64 AvoidObstacleWeight = FixMath.F64.FromFloat(1.0f);

        public FixMath.F64 AvoidNeighborAheadTime = FixMath.F64.FromFloat(1.0f);
        public FixMath.F64 AvoidNeighborWeight = FixMath.F64.FromFloat(1.0f);

        // flocks
        public FixMath.F64 SeparationRadius = FixMath.F64.FromFloat(3.0f);
        public FixMath.F64 SeparationAngle = FixMath.F64.FromFloat(-0.707f);
        public FixMath.F64 SeparationWeight = FixMath.F64.FromFloat(0.0f);

        public FixMath.F64 AlignmentRadius = FixMath.F64.FromFloat(5.0f);
        public FixMath.F64 AlignmentAngle = FixMath.F64.FromFloat(0.707f);
        public FixMath.F64 AlignmentWeight = FixMath.F64.FromFloat(0.0f);

        public FixMath.F64 CohesionRadius = FixMath.F64.FromFloat(4.0f);
        public FixMath.F64 CohesionAngle = FixMath.F64.FromFloat(-0.15f);
        public FixMath.F64 CohesionWeight = FixMath.F64.FromFloat(0.0f);

        // debug toggles
        public bool[] DebugVec3Toggles = new bool[(int)eDebugVec3Item.Count];
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

        public FixMath.F64 QueryLocalBoundaryRadius { get { return Template.AvoidObstacleAheadTime * Template.MaxSpeed + Template.Radius * 3; } }
        public FixMath.F64 QueryLocalNeighborRadius { get { return Template.AvoidNeighborAheadTime * Template.MaxSpeed + Template.Radius * 3; } }

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
        // history avoid info
        private IVehicle.FAvoidObstacleInfo _avoidObstacleInfo = new IVehicle.FAvoidObstacleInfo();
        public IVehicle.FAvoidObstacleInfo AvoidObstacleInfo { get { return _avoidObstacleInfo; } }
        private IVehicle.FAvoidNeighborInfo _avoidNeighborInfo = new IVehicle.FAvoidNeighborInfo();
        public IVehicle.FAvoidNeighborInfo AvoidNeighborInfo { get { return _avoidNeighborInfo; } }

        // local boundary
        public static readonly int  MaxBoundarySegmentNum = 10;
        private BoundarySegement[]  _boundarySegements = null;
        private int                 _boundarySegmentNum = 0;
        private List<IObstacle>     _boundaryObstacles = new List<IObstacle>();
        private AvoidanceQuerySystem _avoidQuerySystem = new AvoidanceQuerySystem();
        private List<UniqueId>      _avoidNeghborIDs = new List<UniqueId>();

        // local neighbors
        private List<IVehicle>      _neighbors = new List<IVehicle>();
        // displacement for neighbor collision
        public FixMath.F64Vec3      Displacement { get; set; }
        // debug forces
        private FixMath.F64Vec3[]   _debugVec3Items = null;

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

        public void OnTemplatePropertyChanged()
        {
            Radius = Template.Radius;
        }

        public virtual void OnCreate()
        {
            _boundarySegements = new BoundarySegement[MaxBoundarySegmentNum];
            Array.Fill(_boundarySegements, default);
            _boundarySegmentNum = 0;

            // initial slow speed
            Speed = FixMath.F64.Zero;
            Radius = Template.Radius;

            if (_proximityToken != null)
                _proximityToken.UpdateForNewPosition(Position);

#if ENABLE_STEER_AGENT_DEBUG
            Debuger = new MovableEntityDebuger();
#endif

            _debugVec3Items = new FixMath.F64Vec3[(int)eDebugVec3Item.Count];
            Array.Fill(_debugVec3Items, FixMath.F64Vec3.Zero);
        }

        public virtual void OnDelete()
        {
            Debuger = null;
        }

        public virtual void OnUpdate(FixMath.F64 inDeltaTime)
        {
            if (!HasEntityState(eEntityState.Moving)) return;
            if (null == targetLocation) return;

            // check if you are too far from the path
            if (null != Pathway)
            {
                var mapPosition = Pathway.MapPointToPath(Position, out var tangent, out var outside);
                var maxOutside = Radius * FixMath.F64.FromFloat(4.0f);
                if (FixMath.F64.Abs(outside) >= maxOutside)
                {
                    isTargetDirty = true;
                }
            }

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
                _proximityToken.FindNeighbors(Position, QueryLocalNeighborRadius, _neighbors);
            }

            // update local boundary
            if (null != LocalBoundaryQuerier)
            {
                _boundarySegmentNum = LocalBoundaryQuerier.QueryBoundaryInCircle(this, QueryLocalBoundaryRadius, _boundarySegements);
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
#endif

            var steerForce = determineCombinedSteering(inDeltaTime);
            if (steerForce != FixMath.F64Vec3.Zero)
            {
                ApplySteeringForce(steerForce, inDeltaTime);

                if (null != _proximityToken)
                    _proximityToken.UpdateForNewPosition(Position);

                _debugVec3Items[(int)eDebugVec3Item.Velocity] = Velocity;
            }

            // 更新避让状态信息
            updateAvoidNeighborInfo();
            updateAvoidObstacleInfo();
        }

        public virtual void OnDraw(bool selected)
        {
            System.Func<FixMath.F64Vec3, FixMath.F64, FixMath.F64Vec2, FixMath.F64Vec2, FixMath.F64Vec3, FixMath.F64, bool> 
                drawVec3 = (vec3, yOffset, inRange, clampRange, color, alpha) => 
            {
                var start = Position; start.Y += yOffset;
                var value = FixMath.F64Vec3.LengthFast(vec3);
                var len = Utilities.GetMappedRangeValueClamped(inRange, clampRange, value);
                var end = start + vec3.GetSafeNormal() * len;
                Util.Draw.drawArrow(annotation, start, end, FixMath.F64Vec2.FromFloat(0.0f, 0.2f), FixMath.F64.FromFloat(2.0f), color);
                return true;
            }; 

            // draw vehicle
            Draw.drawBasic2dCircularVehicle(annotation, this, selected? Colors.Red : Colors.Gray50);
            // drawTrail();
            // draw transform
            Draw.drawAxes(annotation, this, FixMath.F64Vec3.FromFloat(1f, 1f, 1f));
            
            
            if (selected)
            {
                // 1. draw pathway
                if (null != Pathway)
                {
                    var size = FixMath.F64.FromFloat(0.2f);
                    FixMath.F64Vec3? last = null;
                    foreach (var p in Pathway.Points)
                    {
                        annotation.CircleXZ(size, p, Colors.Red, 10);
                        if (null != last)
                        {
                            annotation.Line(last.Value, p, Colors.Green, FixMath.F64.One);
                        }
                        last = p;
                    }
                }

                // 2. draw QueryLocalNeighborRadius
                Draw.drawCircleOrDisk(annotation, QueryLocalNeighborRadius, FixMath.F64Vec3.Up, Position, Colors.Yellow, 10, false, false);

                // 3. draw local boundary segments
                if (_boundaryObstacles.Count > 0)
                {
                    for (var i = 0; i < _boundaryObstacles.Count; ++i)
                    {
                        _boundaryObstacles[i].draw(annotation, false, Colors.Yellow, Position);
                    }
                }

                // 4. draw avoid neighbor check distance
                var testPoint = PredictFuturePosition(Template.AvoidNeighborAheadTime);
                Draw.drawLineAlpha(annotation, Position, testPoint, Colors.Yellow, FixMath.F64.FromFloat(0.4f));

                // 5. draw avoid info
                if (_avoidNeighborInfo.EntityId != UniqueId.InvalidID)
                {
                    var neighbor = EntityManager.GetEntityById(_avoidNeighborInfo.EntityId);
                    if (null != neighbor)
                    {
                        var d = neighbor.Radius.Float * 2;
                        var boxSize = FixMath.F64Vec3.FromFloat(d, 0.1f, d);
                        Draw.drawBoxOutline(annotation, neighbor.Position, FixMath.F64Quat.Identity, boxSize, Colors.OrangeRed, FixMath.F64.One);
                    }
                }

                // 6. draw avoid neighbor detail info
#if false
            if (_avoidNeighborInfo.EntityId != UniqueId.InvalidID)
            {
                var neighbor = EntityManager.GetEntityById(_avoidNeighborInfo.EntityId);
                var relativePosition = neighbor.Position - Position;
                relativePosition.SetYtoZero();
                var distSq = relativePosition.LengthSquared2D();
                var combinedRadius = neighbor.Radius + Radius;
                var combinedRadiusSq = combinedRadius * combinedRadius;

                FixMath.F64Vec3 left, right;

                var leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                left = new FixMath.F64Vec3(relativePosition.X * leg - relativePosition.Z * combinedRadius,
                        FixMath.F64.Zero,
                        relativePosition.X * combinedRadius + relativePosition.Z * leg) / distSq;
                right = new FixMath.F64Vec3(relativePosition.X * leg + relativePosition.Z * combinedRadius,
                        FixMath.F64.Zero,
                        -relativePosition.X * combinedRadius + relativePosition.Z * leg) / distSq;

                // left
                var start = Position;
                var end = Position + left* FixMath.F64.FromFloat(5.0f);
                Util.Draw.drawLine(annotation, start, end, Colors.Red);
                // right
                start = Position;
                end = Position + right * FixMath.F64.FromFloat(5.0f);
                Util.Draw.drawLine(annotation, start, end, Colors.Green);

                // draw side
                if (_avoidNeighborInfo.Side != IVehicle.eAvoidSide.None)
                {
                    var sideDir = _avoidNeighborInfo.Side == IVehicle.eAvoidSide.Left? left : right;
                    start = Position;
                    end = Position + sideDir * FixMath.F64.FromFloat(2.0f);
                    Util.Draw.drawArrow(annotation, start, end, FixMath.F64Vec2.FromFloat(0.0f, 0.2f), FixMath.F64.FromFloat(2.0f), Colors.Yellow);
                }
            }
#else
                {
                    _avoidQuerySystem.DebugDrawGizmos(this, annotation);

                    var boxSize = FixMath.F64Vec3.FromFloat(0.1f, 1.0f, 0.1f);
                    for (var i = 0; i < _avoidNeghborIDs.Count; ++i)
                    {
                        var neighbor = EntityManager.GetEntityById(_avoidNeghborIDs[i]);
                        if (neighbor == null)
                            continue;
                        Draw.drawBoxOutline(annotation, neighbor.Position, FixMath.F64Quat.Identity, boxSize, Colors.Red, FixMath.F64.One);
                    }
                }
#endif

                // 7. draw vectors
                var velocityRange = new FixMath.F64Vec2(FixMath.F64.Zero, MaxSpeed);
                var forceRange = new FixMath.F64Vec2(FixMath.F64.Zero, MaxForce);
                var clampRange = new FixMath.F64Vec2(FixMath.F64.Zero, Radius * 3);

                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.Velocity]) 
                    drawVec3(Velocity, FixMath.F64.FromFloat(1.0f), velocityRange, clampRange, Colors.White, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.ForwardMoveForce]) 
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.ForwardMoveForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray10, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.PathFollowForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.PathFollowForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray20, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.AvoidNeighborForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.AvoidNeighborForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray30, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.AvoidObstacleForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.AvoidObstacleForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray40, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.FlockSeparationForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockSeparationForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray50, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.FlockAligmentForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockAligmentForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray60, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.FlockCohesionForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockCohesionForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray70, FixMath.F64.One);
                if (Template.DebugVec3Toggles[(int)eDebugVec3Item.TotalSteerForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.TotalSteerForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray80, FixMath.F64.One);
            }
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
            // var leakThrough = FixMath.F64.FromFloat(0.1f);

            var steeringForce = FixMath.F64Vec3.Zero;

            // 1.path follow corner point
            var pathFollowForce = FixMath.F64Vec3.Zero;
            {
                if (null != Pathway)
                {
                    var predictionTime = Template.FollowPathAheadTime;
                    pathFollowForce = this.SteerToFollowPath(_pathDirection, predictionTime, Pathway, MaxSpeed, out var currentPathDistance, annotation) * Template.FollowPathWeight;

                    // float pathDistanceOffset = (_pathDirection ? 1 : -1) * predictionTime * Speed;
                    // referencePoint = Pathway.MapPathDistanceToPoint(currentPathDistance + FixMath.F64.FromFloat(0.5f));
                }

                if (pathFollowForce == FixMath.F64Vec3.Zero)
                {
                    pathFollowForce = SteerForSeek(targetLocation.Value) * Template.FollowPathWeight;
                }

                pathFollowForce = pathFollowForce.TruncateLength(MaxForce);

#if ENABLE_STEER_AGENT_DEBUG
                info.targetForce = pathFollowForce * forceScale;
#endif
                steeringForce += pathFollowForce;
            }
            _debugVec3Items[(int)eDebugVec3Item.PathFollowForce] = pathFollowForce;

            // 2.avoid neighbors
            var collisionAvoidance = FixMath.F64Vec3.Zero;
            {
#if false
                // otherwise consider avoiding collisions with others
                List<IVehicle> neighbors = new List<IVehicle>();
                for (int i = 0; i < _neighbors.Count; ++i)
                {
                    var neighbor = _neighbors[i] as MovableEntity;
                    if (null == neighbor || ID == neighbor.ID)
                        continue;
                    //if (neighbor.HasEntityState(eEntityState.Moving))
                    //    continue;
                    neighbors.Add(neighbor);
                }

                // collisionAvoidance = steerToAvoidNeighbors(timeCollisionWithNeighbor, neighbors);
                collisionAvoidance = SteerToAvoidNeighbors(Template.AvoidNeighborAheadTime, neighbors, ref _avoidNeighborInfo);
                collisionAvoidance = collisionAvoidance * Template.AvoidNeighborWeight;
#else
                _avoidNeghborIDs.Clear();
                _avoidQuerySystem.Init(ID, Position.Cast2D(), Radius, Velocity.Cast2D(), Template.AvoidNeighborAheadTime);
                for (var i = 0; i < _neighbors.Count; ++i)
                {
                    var neighbor = _neighbors[i] as MovableEntity;
                    if (null == neighbor || ID == neighbor.ID)
                        continue;
                    if (!ShouldAvoidNeighbor(neighbor))
                        continue;

                    var add = _avoidQuerySystem.AddCircle(neighbor.ID, neighbor.Position.Cast2D(), neighbor.Radius, neighbor.Velocity.Cast2D());
                    if (add) _avoidNeghborIDs.Add(neighbor.ID);
                }

                var avoidDirection = _avoidQuerySystem.QueryAvoidDirection(this, EntityManager.FrameNo, ref _avoidNeighborInfo);
                //// 防止方向突变，加个平滑插值逻辑
                //var percent = FixMath.F64.Clamp01(elapsedTime / FixMath.F64.FromFloat(0.2f));
                //if (avoidDirection == FixMath.F64Vec3.Zero)
                //{
                //    _avoidNeighborForceWeight = FixMath.F64.Lerp(_avoidNeighborForceWeight, FixMath.F64.Zero, percent);
                //}
                //else
                //{
                //    _avoidNeighborForceWeight = Template.AvoidNeighborWeight;
                //    _avoidNeighborDirection = avoidDirection;
                //}
                var lateral = Vector3Helpers.PerpendicularComponent(avoidDirection, Forward);
                collisionAvoidance = FixMath.F64Vec3.NormalizeFast(lateral) * MaxForce * Template.AvoidNeighborWeight;
#endif

#if ENABLE_STEER_AGENT_DEBUG
                info.avoidNeighborFoce = collisionAvoidance * forceScale;
#endif
                steeringForce += collisionAvoidance;
            }
            _debugVec3Items[(int)eDebugVec3Item.AvoidNeighborForce] = collisionAvoidance;

            // 3.avoid obstacles
            var obstacleAvoidance = FixMath.F64Vec3.Zero;
            {
                _boundaryObstacles.Clear();
                // var obstacles = new List<IObstacle>();
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
                    obstacleAvoidance = SteerToAvoidObstacles(Template.AvoidObstacleAheadTime, _boundaryObstacles, ref _avoidObstacleInfo) * Template.AvoidObstacleWeight;

#if ENABLE_STEER_AGENT_DEBUG
                    info.obstacleForce = obstacleAvoidance * forceScale;
#endif
                }

                steeringForce += obstacleAvoidance;
            }
            _debugVec3Items[(int)eDebugVec3Item.AvoidObstacleForce] = obstacleAvoidance;

            // 
            if (collisionAvoidance != FixMath.F64Vec3.Zero ||
                obstacleAvoidance != FixMath.F64Vec3.Zero)
            {
                var forwardMoveForce = FixMath.F64Vec3.Zero;
                {
                    forwardMoveForce = Forward * MaxForce * Template.ForwardMoveWeight;
                    steeringForce += forwardMoveForce;
                }
                _debugVec3Items[(int)eDebugVec3Item.ForwardMoveForce] = forwardMoveForce;
            }

            // flock
            if (obstacleAvoidance == FixMath.F64Vec3.Zero)
            { 
                var flockForce = steerToFlock(elapsedTime);

                steeringForce += flockForce * forceScale;
            }

            var totalForce = steeringForce.SetYtoZero();
#if ENABLE_STEER_AGENT_DEBUG
	        info.steerForce = totalForce;
#endif
            _debugVec3Items[(int)eDebugVec3Item.TotalSteerForce] = totalForce;
            return totalForce;
        }

        FixMath.F64Vec3 steerToFlock(FixMath.F64 elapsedTime)
        {
            // determine each of the three component behaviors of flocking
            var separation = SteerForSeparation(Template.SeparationRadius,
                                                Template.SeparationAngle,
                                                _neighbors);
            var alignment = SteerForAlignment(Template.AlignmentRadius,
                                              Template.AlignmentAngle,
                                              _neighbors);
            var cohesion = SteerForCohesion(Template.CohesionRadius,
                                            Template.CohesionAngle,
                                            _neighbors);

            // apply weights to components (save in variables for annotation)
            var separationForce = separation * Template.SeparationWeight;
            var alignmentForce = alignment * Template.AlignmentWeight;
            var cohesionForce = cohesion * Template.CohesionWeight;

            _debugVec3Items[(int)eDebugVec3Item.FlockSeparationForce] = separationForce;
            _debugVec3Items[(int)eDebugVec3Item.FlockAligmentForce] = alignmentForce;
            _debugVec3Items[(int)eDebugVec3Item.FlockCohesionForce] = cohesionForce;
            return separationForce + alignmentForce + cohesionForce;
        }

        public override FixMath.F64Vec3 GetAvoidObstacleDirection(IObstacle obstacle, ref PathIntersection pathIntersection, ref IVehicle.FAvoidObstacleInfo info) 
        {
            // 选择最优避让方向规则：
            // (1) 和上一次选择side保持一致, 如果为none，则检查_avoidNeighborInfo的side，与其保持一致

            var det = Vector3Helpers.Det(pathIntersection.surfaceNormal.Cast2D(), pathIntersection.steerHint.Cast2D());

            var side = IVehicle.eAvoidSide.Left;
            if (info.Side == IVehicle.eAvoidSide.None)
            { 
                if (_avoidNeighborInfo.IsValid)
                {
                    side = _avoidNeighborInfo.Side;
                }
                else
                {
                    // 根据Intersection方向来选择
                    side = det >= 0? IVehicle.eAvoidSide.Left : IVehicle.eAvoidSide.Right;
                }
            }
            else
            {
                side = info.Side;
            }

            info.SetAvoidInfo(EntityManager.FrameNo, obstacle, side);

            var isReflect = (side == IVehicle.eAvoidSide.Left && det < 0) || (side == IVehicle.eAvoidSide.Right && det > 0);
            ref var steerHint = ref pathIntersection.steerHint;
            ref var normal = ref pathIntersection.surfaceNormal;
            var direction = isReflect? (2 * steerHint.Dot(normal) * normal - steerHint) : steerHint; 
            return direction.GetSafeNormal();
        }

        public override FixMath.F64Vec3 GetAvoidNeighborDirection(IVehicle threat, PathIntersection? intersection, ref IVehicle.FAvoidNeighborInfo info) 
        {
            // 选择最优避让方向规则：
            // (1) 与desiredVelocity方向最接近
            // (2) 和上一次选择side保持一致, 如果为none，则检查_avoidObstacleInfo的side，与其保持一致
            // (3) 被避让单位是否在避让自己，选择与其保持一致的避让方向（比如都选左，或者右）
            // (4) 检查选择的避让方向是否会撞墙
            if (threat is MovableEntity entity)
            {
                var relativePosition = threat.Position - Position;
                relativePosition.SetYtoZero();
                if (relativePosition == FixMath.F64Vec3.Zero) 
                    return FixMath.F64Vec3.Zero;

                var distSq = relativePosition.LengthSquared();
                var combinedRadius = threat.Radius + Radius;
                var combinedRadiusSq = combinedRadius * combinedRadius;

                FixMath.F64Vec3 left, right;

                var leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                // 基于以relativePosition为Forward的坐标系
                left = new FixMath.F64Vec3(relativePosition.X * leg - relativePosition.Z * combinedRadius,
                        FixMath.F64.Zero,
                        relativePosition.X * combinedRadius + relativePosition.Z * leg) / distSq;
                right = new FixMath.F64Vec3(relativePosition.X * leg + relativePosition.Z * combinedRadius,
                        FixMath.F64.Zero,
                        -relativePosition.X * combinedRadius + relativePosition.Z * leg) / distSq;

                var side = IVehicle.eAvoidSide.Left;
                if (info.Side == IVehicle.eAvoidSide.None)
                {
                    // 选择和对方一致的避让方向
                    if (entity.AvoidNeighborInfo.EntityId == ID)
                    {
                        side = entity.AvoidNeighborInfo.Side;
                    }
                    else if (_avoidObstacleInfo.IsValid)
                    {
                        side = _avoidObstacleInfo.Side;
                    }
                    else
                    {
                        // 选择不撞墙的方向
                        var minDistanceToCollision = Template.AvoidObstacleAheadTime * Speed;
                        var testDirs = new FixMath.F64Vec3[] { left, right };
                        var testSides = new IVehicle.eAvoidSide[] { IVehicle.eAvoidSide.Left, IVehicle.eAvoidSide.Right };

                        var movable = new SimpleVehicle();
                        movable.Position = Position;
                        movable.Radius = Radius;

                        for (var dir = 0; dir < testDirs.Length; ++dir)
                        {
                            var forward = testDirs[dir];
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
                    side = info.Side;
                }

                // 根据side选择避让方向
                info.SetAvoidInfo(EntityManager.FrameNo, entity.ID, side);
                return side == IVehicle.eAvoidSide.Left? left : right;
            }
            return FixMath.F64Vec3.Zero; 
        }

        static FixMath.F64 IRECTION_ANGLE_THRESHOLD = FixMath.F64.DegToRad(FixMath.F64.FromFloat(45.0f));
        static FixMath.F64 DIRECTION_ANGLE_THRESHOLD_COS = FixMath.F64.CosFast(IRECTION_ANGLE_THRESHOLD);
        public override bool ShouldAvoidNeighbor(IVehicle threat) 
        { 
            // 规则：
            // (1) 静止的单位需要避让
            // (2) 移动方向同向的不避让，移动方向反向的需避让
            if (threat is MovableEntity entity)
            {
                if (!entity.HasEntityState(eEntityState.Moving))
                {
                    return true;
                }

                var dir0 = Velocity.GetSafeNormal();
                var dir1 = entity.Velocity.GetSafeNormal();
                var dot = dir0.Dot(dir1);
                if (dot < DIRECTION_ANGLE_THRESHOLD_COS)
                {
                    return true;
                }
            }
            return false; 
        }

        // 更新避让neighbor信息，并在合适的时机重置信息
        void updateAvoidNeighborInfo()
        {
            if (!_avoidNeighborInfo.EntityId.IsValid())
            {
                _avoidNeighborInfo.Reset();
                return;
            }

            // 超过一定帧数没有需要避让的单位，则重置避让信息
            var deltaFrame = EntityManager.FrameNo - _avoidNeighborInfo.FrameNo;
            if (deltaFrame > 5)
            {
                _avoidNeighborInfo.Reset();
            }
        }

        void updateAvoidObstacleInfo()
        {
            if (null == _avoidObstacleInfo.Obstacle)
            {
                _avoidObstacleInfo.Reset();
            }

            var deltaFrame = EntityManager.FrameNo - _avoidObstacleInfo.FrameNo;
            if (deltaFrame > 5)
            {
                _avoidObstacleInfo.Reset();
            }
        }

        // 返回避让邻近单位位移量
        static FixMath.F64 COLLISION_RESOLVE_FACTOR = FixMath.F64.FromFloat(0.7f);
        static FixMath.F64 DISTANCE_EPLISION = FixMath.F64.FromFloat(0.001f);
        public FixMath.F64Vec3 ResolveCollisionWithNeighbors()
        {
            var Disp = FixMath.F64Vec3.Zero;
            var w = FixMath.F64.Zero;

            for (int i = 0; i < _neighbors.Count; ++i)
            {
                var nei = _neighbors[i] as MovableEntity;
                if (null == nei || ID == nei.ID)
                    continue;

                // 检查距离
                var Diff = Position - nei.Position;
                Diff.Y = FixMath.F64.Zero;
                var DistSq = Diff.LengthSquared();
                var Radii = Radius + nei.Radius;

                if (DistSq > Radii * Radii)
                    continue;

                var Dist = FixMath.F64.Sqrt(DistSq);
                var Pen = FixMath.F64.FromFloat(0.01f);
                if (Dist < DISTANCE_EPLISION)
                {
                    if (ID.Id < nei.ID.Id)
                    {
                        Diff = new FixMath.F64Vec3(-Velocity.Z, FixMath.F64.Zero, Velocity.X);
                    }
                    else
                    {
                        Diff = new FixMath.F64Vec3(Velocity.Z, FixMath.F64.Zero, -Velocity.X);
                    }
                }
                else
                {
                    Pen = (FixMath.F64.One / Dist) * (Radii - Dist) * FixMath.F64.Half * COLLISION_RESOLVE_FACTOR;
                }
                var Mtd = Pen * Diff;
                // minimum translation distance to push balls apart after intersecting
                Disp += Mtd;

                w += FixMath.F64.One;
            }

            if (w > FixMath.F64.Zero)
            {
                var iw = FixMath.F64.One / w;
                Disp *= iw;
            }
            return Disp;
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
