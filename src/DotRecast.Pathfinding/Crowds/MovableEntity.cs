using System;
using SharpSteer2;
using SharpSteer2.Database;
using System.Collections.Generic;
using SharpSteer2.Pathway;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using Pathfinding.Crowds;
using Pathfinding.Util;
using Pathfinding.Crowds.AvoidStrategy;
using Pathfinding.Crowds.MoveStrategy;

namespace Pathfinding.Crowds
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
    public class TMovableEntityTemplate : TEntityTemplate
    {
        // stop move radius
        public FixMath.F64 StopMoveRadius = FixMath.F64.FromFloat(1.5f);
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
        public FixMath.F64 AvoidObstacleWeight = FixMath.F64.FromFloat(0.3f);

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

    public class MovableEntity : SimpleVehicle, ICrowdEntityActor
    {
        // ICrowdEntityActor interface
        public TEntityTemplate Template { get; set; }
        public IMovableEntityManager EntityManager { get; set; }
        // Entity unique id
        public UniqueId ID { get; set; }
        public Volatile.VoltBody PhysicsBody { get; set; }
        public int GroupMask { get; set; }
        public int GroupsToAvoid { get; set; }
        public void SetPosition(FixMath.F64Vec3 position) { Position = position; }
        public FixMath.F64Vec3 GetPosition() { return Position; }
        public void SetRotation(FixMath.F64Quat rotation)
        {
            Forward = rotation * Vector3Helpers.Forward;
            Side = rotation * Vector3Helpers.Right;
            Up = rotation * Vector3Helpers.Up;
        }
        public FixMath.F64Quat GetRotation() 
        {
            return FixMath.F64Quat.LookRotation(Forward, FixMath.F64Vec3.Up);
        }
        // End

        // 单位状态
        public enum  eEntityState
        {
	        Moving = 0,
            Attack,
        };

        // a pointer to this boid's interface object for the proximity database
        private ITokenForProximityDatabase<IVehicle> _proximityToken;

        public FixMath.F64 QueryLocalBoundaryRadius { 
            get {
                var template = Template as TMovableEntityTemplate;
                return template.AvoidObstacleAheadTime * template.MaxSpeed + template.Radius * 3; 
            } 
        }
        public FixMath.F64 QueryLocalNeighborRadius { 
            get {
                var template = Template as TMovableEntityTemplate;
                return template.AvoidNeighborAheadTime * template.MaxSpeed + template.Radius * 3; 
            } 
        }

        public ILocalBoundaryQuerier LocalBoundaryQuerier { get; private set; }

        public IPathwayQuerier PathwayQuerier { get; private set; }

        // Move destination
        protected bool isTargetDirty { get; private set; }
        protected FixMath.F64Vec3? targetLocation = null;

        static FixMath.F64 DEFAULT_INITIAL_MOVE_SPEED = FixMath.F64.FromFloat(0.1f);
        public FixMath.F64Vec3? TargetLocation { 
            get
            {
                return targetLocation;
            }
            
            set 
            {
                if (value != null)
                {
                    isTargetDirty = null == targetLocation || FixMath.F64Vec3.DistanceFast(targetLocation.Value, value.Value) > FixMath.F64.Two;
                    if (isTargetDirty) 
                    {
                        targetLocation = value;
                        if (null != targetLocation)
                        {
                            var template = Template as TMovableEntityTemplate;
                            // Give a little initial speed to prevent the position from being unchanged due to the speed being 0 in the PredictFuturePosition function,
                            // causing some division by 0 exceptions
                            Speed = template.MaxSpeed * FixMath.F64.Half;
                            SetEntityState(eEntityState.Moving);
                        }
                        else
                        {
                            Speed = FixMath.F64.Zero;
                            ClearEntityState(eEntityState.Moving);
                        }
                    }
                }
                else
                {
                    Speed = FixMath.F64.Zero;
                    ClearEntityState(eEntityState.Moving);
                }

                targetLocation = value;
            }
        }

        // Debug
        public MovableEntityDebuger Debuger { get; private set; }
        public IAnnotationService Annotation { get { return annotation; } }

        // route for path following (waypoints and legs)
        public PolylinePathway Pathway { get; private set; }
        
        // local boundary
        public static readonly int  MaxBoundarySegmentNum = 10;
        private BoundarySegement[]  _boundarySegements = null;
        private int                 _boundarySegmentNum = 0;
        private List<IObstacle>     _boundaryObstacles = new List<IObstacle>();
        public List<IObstacle>      BoundaryObstacles { get { return _boundaryObstacles; } }

        // local neighbors
        private List<IVehicle>      _neighbors = new List<IVehicle>();
        public List<IVehicle>       Neighbors { get { return _neighbors; } }
        // displacement for neighbor collision
        public FixMath.F64Vec3      Displacement { get; set; }

        // debug forces
        private FixMath.F64Vec3[]   _debugVec3Items = null;

        // state bits
        private uint                _stateBitsValue = 0;

        // move strategies
        private int                 _moveStrategyIndex = -1;
        private IMoveStrategy[]     _moveStrategies = null;

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
            var template = Template as TMovableEntityTemplate; 
            Radius = template.Radius;

            // 重新创建移动策略
            _moveStrategyIndex = -1;
            _moveStrategies = new IMoveStrategy[] {
                new IdleMoveStrategy(this),
                new FollowPathMoveStrategy(this),
                new AttackMoveStrategy(this),
            };
        }

        public virtual void OnCreate()
        {
            var template = Template as TMovableEntityTemplate;

            _boundarySegements = new BoundarySegement[MaxBoundarySegmentNum];
            Array.Fill(_boundarySegements, default);
            _boundarySegmentNum = 0;

            // initial slow speed
            Speed = FixMath.F64.Zero;
            Radius = template.Radius;
            MaxSpeed = template.MaxSpeed;
            MaxForce = template.MaxForce;

            if (_proximityToken != null)
                _proximityToken.UpdateForNewPosition(Position);

            // 初始化移动策略
            _moveStrategies = new IMoveStrategy[] {
                new IdleMoveStrategy(this),
                new FollowPathMoveStrategy(this),
                new AttackMoveStrategy(this),
            };

#if ENABLE_STEER_AGENT_DEBUG
            Debuger = new MovableEntityDebuger();
#endif

            _debugVec3Items = new FixMath.F64Vec3[(int)eDebugVec3Item.Count];
            Array.Fill(_debugVec3Items, FixMath.F64Vec3.Zero);
        }

        public virtual void OnDelete()
        {
            _moveStrategies = null;
            ICrowdEntityActor.DestroyPhysicsBody(this, PhysicsBody);
            PhysicsBody = null;
            Debuger = null;
        }

        public void OnCreatePhysicsState()
        {
            var physicsWorld = EntityManager.PhysicsWorld;
            // 创建Shape
            var shape = physicsWorld.CreateCircleWorldSpace(Position.ToVoltVec2(), Radius.ToF64());
            // 创建Body
            PhysicsBody = ICrowdEntityActor.CreatePhysicsBody(this, new Volatile.VoltShape[] { shape });
        }

        public void OnDestroyPhysicsState()
        {
            // 销毁物理体
            ICrowdEntityActor.DestroyPhysicsBody(this, PhysicsBody);
            PhysicsBody = null;
        }

        // 是否是物理驱动模式
        private bool _phyiscsDrivenMode = false;
        public virtual void OnPrePhysics()
        {
            // 设置逻辑层位置到物理层
            if (null != PhysicsBody)
            {
                var position = GetPosition();
                var rotation = GetRotation();


                if (_phyiscsDrivenMode)
                {
                    PhysicsBody.LinearVelocity = Velocity.ToVoltVec2();
                }
                else
                {
                    if (FixMath.F64Quat.ToEulerAnglesDegree(rotation, out var Eluer))
                    {
                        PhysicsBody.Set(position.ToVoltVec2(), FixMath.F64.DegToRad(Eluer.Y).ToF64());
                    }
                    else
                    {
                        PhysicsBody.Set(position.ToVoltVec2(), FixMath.NET.Fix64.Zero);
                    }

                    PhysicsBody.LinearVelocity = Volatile.VoltVector2.zero;
                }
            }
        }

        public virtual void OnPostPhysics() 
        {
            if (null != PhysicsBody)
            {
                // Idle和Attack状态的单位不允许被挤开
                if (HasEntityState(eEntityState.Moving))
                {
                    var NewPosition = PhysicsBody.Position.ToVec3(Position.Y);
                    // var NewRotation = FixMath.F64Quat.FromYawPitchRoll(
                    //    FixMath.F64.DegToRad(PhysicsBody.Angle.ToF64()),
                    //    FixMath.F64.Zero,
                    //    FixMath.F64.Zero);
                    // var NewVelocity = PhysicsBody.LinearVelocity.ToVec3(FixMath.F32.Zero);

                    SetPosition(NewPosition);
                    // SetRotation(NewRotation);
                    // SetVelocity(NewVelocity);
                }
            }
        }

        public virtual void OnUpdate(FixMath.F64 inDeltaTime)
        {
            if (null == targetLocation) return;

            var template = Template as TMovableEntityTemplate;

            // check in stop radius
            if (HasEntityState(eEntityState.Moving))
            {
                var distance = FixMath.F64Vec3.DistanceFast(Position, targetLocation.Value);
                if (distance < template.StopMoveRadius)
                {
                    Speed = FixMath.F64.Zero;
                    ClearEntityState(eEntityState.Moving);
                }
            }
            else
            {
                Speed = FixMath.F64.Zero;
                return;
            }

            // 更新移动策略
            updateMoveStrategy();

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
                // create boundary obstacles
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
            }

            // determine steering force
#if ENABLE_STEER_AGENT_DEBUG
            if (HasEntityState(eEntityState.Moving))
            {
                ref var info = ref Debuger.EntityInfoBuff.Alloc(EntityManager.FrameNo);
                info.maxSpeed = MaxSpeed;
                info.radius = Radius;
                info.position = Position;
                info.velocity = Velocity;
                info.forward = Forward;
                info.side = Side;
                info.up = Up;
            }
#endif

            var steerForce = determineCombinedSteering(inDeltaTime);
            steerForce = steerForce.SetYtoZero();
            if (steerForce != FixMath.F64Vec3.Zero)
            {
                ApplySteeringForce(steerForce, inDeltaTime);

                if (null != _proximityToken)
                    _proximityToken.UpdateForNewPosition(Position);

                _debugVec3Items[(int)eDebugVec3Item.Velocity] = Velocity;
            }
        }

        public virtual void OnDraw(bool selected)
        {
            var template = Template as TMovableEntityTemplate;

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
                // Draw.drawCircleOrDisk(annotation, QueryLocalNeighborRadius, FixMath.F64Vec3.Up, Position, Colors.Yellow, 10, false, false);

                // 3. draw local boundary segments
                if (_boundaryObstacles.Count > 0)
                {
                    for (var i = 0; i < _boundaryObstacles.Count; ++i)
                    {
                        _boundaryObstacles[i].draw(annotation, false, Colors.Yellow, Position);
                    }
                }

                // 4. draw avoid neighbor check distance
                var testPoint = PredictFuturePosition(template.AvoidNeighborAheadTime);
                Draw.drawLineAlpha(annotation, Position, testPoint, Colors.Yellow, FixMath.F64.FromFloat(0.4f));

                // 5. draw move strategy infos
                if (-1 != _moveStrategyIndex)
                {
                    _moveStrategies[_moveStrategyIndex].DrawGizmos(this);
                }

                // 6. draw vectors
                var velocityRange = new FixMath.F64Vec2(FixMath.F64.Zero, MaxSpeed);
                var forceRange = new FixMath.F64Vec2(FixMath.F64.Zero, MaxForce);
                var clampRange = new FixMath.F64Vec2(FixMath.F64.Zero, Radius * 3);

                if (template.DebugVec3Toggles[(int)eDebugVec3Item.Velocity]) 
                    drawVec3(Velocity, FixMath.F64.FromFloat(1.0f), velocityRange, clampRange, Colors.White, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.ForwardMoveForce]) 
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.ForwardMoveForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray10, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.PathFollowForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.PathFollowForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray20, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.AvoidNeighborForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.AvoidNeighborForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray30, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.AvoidObstacleForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.AvoidObstacleForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray40, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.FlockSeparationForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockSeparationForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray50, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.FlockAligmentForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockAligmentForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray60, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.FlockCohesionForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.FlockCohesionForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray70, FixMath.F64.One);
                if (template.DebugVec3Toggles[(int)eDebugVec3Item.TotalSteerForce])
                    drawVec3(_debugVec3Items[(int)eDebugVec3Item.TotalSteerForce], FixMath.F64.FromFloat(0.0f), forceRange, clampRange, Colors.Gray80, FixMath.F64.One);
            }
        }

        // compute combined steering force: move forward, avoid obstacles
        // or neighbors if needed, otherwise follow the path and wander
        FixMath.F64Vec3 determineCombinedSteering(FixMath.F64 elapsedTime)
        {
            if (-1 != _moveStrategyIndex)
            {
                var moveStrategy = _moveStrategies[_moveStrategyIndex];
                return moveStrategy.OnUpdate(this, elapsedTime);
            }

            return FixMath.F64Vec3.Zero;
        }

        void updateMoveStrategy()
        {
            for (var i = 0; i < _moveStrategies.Length; ++i)
            {
                var strategy = _moveStrategies[i];
                if (null != strategy && strategy.Condition(this))
                {
                    if (_moveStrategyIndex != i)
                    {
                        if (-1 != _moveStrategyIndex) _moveStrategies[_moveStrategyIndex].OnStop(this);
                        _moveStrategies[i].OnStart(this);
                        _moveStrategyIndex = i;
                        break;
                    }
                }
            }
        }

        public override FixMath.F64Vec3 GetAvoidObstacleDirection(IObstacle obstacle, ref PathIntersection pathIntersection, ref IVehicle.FAvoidObstacleInfo info) 
        {
            // 选择最优避让方向规则：
            // (1) 和上一次选择side保持一致, 如果为none，则检查_avoidNeighborInfo的side，与其保持一致

            var det = Vector3Helpers.Det(pathIntersection.surfaceNormal.Cast2D(), pathIntersection.steerHint.Cast2D());

            var side = IVehicle.eAvoidSide.Left;
            if (info.Side == IVehicle.eAvoidSide.None)
            { 
                if (info.IsValid)
                {
                    side = info.Side;
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
            var template = Template as TMovableEntityTemplate;

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
                    if (info.EntityId == ID)
                    {
                        side = info.Side;
                    }
                    else if (info.IsValid)
                    {
                        side = info.Side;
                    }
                    else
                    {
                        // 选择不撞墙的方向
                        var minDistanceToCollision = template.AvoidObstacleAheadTime * Speed;
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

        static FixMath.F64 DIRECTION_ANGLE_THRESHOLD = FixMath.F64.DegToRad(FixMath.F64.FromFloat(45.0f));
        static FixMath.F64 DIRECTION_ANGLE_THRESHOLD_COS = FixMath.F64.CosFast(DIRECTION_ANGLE_THRESHOLD);
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

        // 返回避让邻近单位位移量
        static FixMath.F64 COLLISION_RESOLVE_FACTOR = FixMath.F64.FromFloat(0.7f);
        static FixMath.F64 COLLISION_RADIUS_SCALE = FixMath.F64.FromDouble(1.2f);
        static FixMath.F64 DISTANCE_EPLISION = FixMath.F64.FromFloat(0.001f);
        public FixMath.F64Vec3 ResolveCollisionWithNeighbors()
        {
            var isIdle = !HasEntityState(eEntityState.Moving);
            if (isIdle) return FixMath.F64Vec3.Zero;

            // 碰撞半径比单位半径稍微大一点，尽量防止2个单位穿插
            var Disp = FixMath.F64Vec3.Zero;
            var w = FixMath.F64.Zero;
            var R = Radius * COLLISION_RADIUS_SCALE;

            for (int i = 0; i < _neighbors.Count; ++i)
            {
                var nei = _neighbors[i] as MovableEntity;
                if (null == nei || ID == nei.ID)
                    continue;

                // Idle的单位只和Idle单位处理碰撞
                //if (isIdle && nei.HasEntityState(eEntityState.Moving))
                //{
                //    continue;
                //}

                var neiR = nei.Radius * COLLISION_RADIUS_SCALE;

                // 检查距离
                var Diff = Position - nei.Position;
                Diff.Y = FixMath.F64.Zero;
                var DistSq = Diff.LengthSquared();
                var Radii = R + neiR;

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
