using System.Collections.Generic;
using System.IO;
using FixMath;
using Pathfinding.Crowds;
using Pathfinding.Triangulation.Data;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Database;
using SharpSteer2.Helpers;
using Volatile;
using static Pathfinding.Crowds.IMovableEntityManager;

namespace Pathfinding.Crowds
{
    public class MovableEntityManager : IMovableEntityManager, IContactListener
    {
        #region IContactListener
        List<(UniqueId, UniqueId)> ContactPairs = new List<(UniqueId, UniqueId)> ();
        public void BeginContact(ContactInfo contact)
        {
            if (contact != null) 
            {
                var EntityA = contact.m_bodyA.UserData as ICrowdEntityActor;              
                var EntityB = contact.m_bodyB.UserData as ICrowdEntityActor;
                if (null != EntityA && null != EntityB)
                {
                    var Index = ContactPairs.FindIndex(e => (e.Item1 == EntityA.ID &&  e.Item2 == EntityB.ID || e.Item1 == EntityB.ID && e.Item2 == EntityA.ID));
                    if (-1 == Index) ContactPairs.Add((EntityA.ID, EntityB.ID));
                }
            }
        }

        public void EndContact(ContactInfo contact)
        {
            if (contact != null)
            {
                var EntityA = contact.m_bodyA.UserData as ICrowdEntityActor;
                var EntityB = contact.m_bodyB.UserData as ICrowdEntityActor;
                if (null != EntityA && null != EntityB)
                {
                    var Index = ContactPairs.FindIndex(e => (e.Item1 == EntityA.ID && e.Item2 == EntityB.ID || e.Item1 == EntityB.ID && e.Item2 == EntityA.ID));
                    ContactPairs.RemoveAt(Index);
                }
            }
        }
        #endregion

        public const float WORLD_RADIUS = 50;
        public const int TICK_FRAME_PERSECOND = 20;
        public FixMath.F64 TICK_FRAME_DELTATIME = FixMath.F64.FromDouble(1.0 / TICK_FRAME_PERSECOND);
        public const int PHYSICS_SUBSTEP_NUM = 5;
        public FixMath.F64 PHYSICS_FRAME_DELTATIME = FixMath.F64.FromDouble(1.0 / (TICK_FRAME_PERSECOND * PHYSICS_SUBSTEP_NUM));

        private FixMath.F64 _tickElapsedTime = FixMath.F64.Zero;

        // 记录单位Id到索引对象的映射
        Dictionary<UniqueId, int> EntityId2Index = null;
        List<ICrowdEntityActor> Entities = null;

        // pointer to database used to accelerate proximity queries
        private IProximityDatabase<IVehicle> _pd;

        // for purposes of demonstration, allow cycling through various
        // types of proximity databases.  this routine is called when the
        // Demo user pushes a function key.
        // rect area: (center - extent * 0.5) - (center + extent * 0.5)
        private void AllocPD(FixMath.F64Vec3 center, FixMath.F64Vec3 extent, int div)
        {
            // allocate new PD
            var DIV = FixMath.F64.FromFloat(div);
            var divisions = new FixMath.F64Vec3(DIV, DIV, DIV);
            var DIAMETER = FixMath.F64.FromFloat(WORLD_RADIUS * 1.1f * 2);
            var dimensions = new FixMath.F64Vec3(DIAMETER, DIAMETER, DIAMETER);
            _pd = new LocalityQueryProximityDatabase<IVehicle>(center, dimensions, divisions);
        }

        private bool _initialized = false;
        private uint _entityIdCounter = 1;

        // physics world
        private Volatile.VoltWorld _physicsWorld = null;

        // logic map
        private Map _map = null;

        public int FrameNo { get; set; }

        public Volatile.VoltWorld PhysicsWorld { get { return _physicsWorld; } }

        public Map Map { get { return _map; } }
        public IAnnotationService AnnotationService { get; set; }

        private FInitializeParams initializeParams;
        private Recorder _recorder = null;
        // 标记当前是否是录像控制模式（不允许执行任何逻辑，全部由replay来驱动逻辑）
        private bool _isReplayControlMode = false;

        public UniqueId CreateEntity(CreateEntityParams inParams)
        {
            if (_isReplayControlMode)
                return UniqueId.InvalidID;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationCreateEntity(inParams));
            }

            ICrowdEntityActor entityActor = null;
            if (inParams.Template is TMovableEntityTemplate)
            {
                entityActor = new MovableEntity(this, 
                    inParams.PathwayQuerier?? _map, 
                    inParams.LocalBoundaryQuerier?? _map, 
                    inParams.AnnotationService?? AnnotationService);
            }
            else if (inParams.Template is TUnMovableEntityTemplate)
            {
                entityActor = new UnMovableEntity(this);
            }

            if (null == entityActor)
                return UniqueId.InvalidID;

            entityActor.ID = inParams.EntityId.IsValid()? inParams.EntityId : new UniqueId(_entityIdCounter++);

            // map EntityId and Entity
            Entities.Add(entityActor);
            var entityIndex = Entities.Count - 1;
            EntityId2Index.Add(entityActor.ID, entityIndex);
            // Set template
            entityActor.Template = inParams.Template;

            // position
            entityActor.SetPosition(inParams.SpawnPosition);
            // rotation
            entityActor.SetRotation(inParams.SpawnRotation);

            if (entityActor is MovableEntity entity)
            {
                // add to pd
                entity.NewPD(_pd);
            }

            entityActor.OnCreate();
            if (null != _physicsWorld) entityActor.OnCreatePhysicsState();
            return entityActor.ID;
        }

        public bool DeleteEntity(UniqueId inEntityId)
        {
            if (_isReplayControlMode)
                return false;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationDeleteEntity(inEntityId));
            }

            if (EntityId2Index.TryGetValue(inEntityId, out var findIndex))
	        {
                var slotIndex = findIndex;
                var entity = Entities[slotIndex];

                if (entity is MovableEntity movable) movable.NewPD(null);
                if (null != _physicsWorld) entity.OnDestroyPhysicsState();
                entity.OnDelete();

                if ((slotIndex + 1) < Entities.Count)
                {
                    var lastSlotIndex = Entities.Count - 1;
                    var lastEntity = Entities[lastSlotIndex];
                    var lastEntityId = lastEntity.ID;
                    EntityId2Index[lastEntityId] = slotIndex;
                }
                Utilities.RemoveAtSwap(Entities, slotIndex);
                EntityId2Index.Remove(inEntityId);
                return true;
            }
            return false;
        }

        public bool MoveEntity(UniqueId inEntityId, FixMath.F64Vec3? target)
        {
            if (_isReplayControlMode)
                return false;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationMoveEntity(inEntityId, target));
            }

            var entity = GetEntityById(inEntityId);
            if (entity is MovableEntity movable)
            {
                movable.TargetLocation = target;
                return true;
            }
            return false;
        }

        public bool SetEntityParams(
            UniqueId inEntityId, 
            FixMath.F64? radius = null, 
            FixMath.F64? maxSpeed = null, 
            FixMath.F64? maxForce = null, 
            int? groupMask = null,
            int? groupToAvoid = null)
        {
            if (_isReplayControlMode)
                return false;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationSetEntityParams(inEntityId, radius, maxSpeed, maxForce));
            }

            var entity = GetEntityById(inEntityId);
            if (entity is MovableEntity movable)
            {
                if (null != radius) movable.Radius = radius.Value;
                if (null != maxSpeed) movable.MaxSpeed = maxSpeed.Value;
                if (null != maxForce) movable.MaxForce = maxForce.Value;
                if (null != groupMask) movable.GroupMask = groupMask.Value;
                if (null != groupToAvoid) movable.GroupsToAvoid = groupToAvoid.Value;
                return true;
            }
            return false;
        }

        public ICrowdEntityActor GetEntityById(UniqueId inEntityId)
        {
            if (EntityId2Index.TryGetValue(inEntityId, out var entityIndex))
            {
                return Entities[entityIndex] as MovableEntity;
            }
            return null;
        }
        public ICrowdEntityActor[] GetEntitiesInCircle(FixMath.F64Vec3 inCenter, FixMath.F64 inRadius)
        {
            var createEntityParams = new CreateEntityParams() 
            {
                SpawnPosition = inCenter,

                Template = new TMovableEntityTemplate(),
            };
            var entityId = CreateEntity(createEntityParams);
            if (!entityId.IsValid()) return null;

            var entity = GetEntityById(entityId) as MovableEntity;
            var result = entity.FindNeighbors(inRadius);
            DeleteEntity(entityId);

            if (result == null) return null;

            var entities = new List<MovableEntity>();
            foreach ( var neighbor in result ) 
            {
                if (neighbor is MovableEntity neiEntity) entities.Add(neiEntity);
            }
            return entities.ToArray();
        }

        public bool Initialize(FInitializeParams inParams)
        {
            if (_isReplayControlMode)
                return false;

            if (_initialized) return true;

            // Triangulation
            Vertex.INC = 0;
            Face.INC = 0;
            Edge.INC = 0;
            ConstraintSegment.INC = 0;
            ConstraintShape.INC = 0;
            Obstacle.INC = 0;
            //

            EntityId2Index = new Dictionary<UniqueId, int>();
            Entities = new List<ICrowdEntityActor>();

            FrameNo = 1;
            _entityIdCounter = 1;
            _tickElapsedTime = FixMath.F64.Zero;

            var center = (inParams.MapBoundsMin + inParams.MapBoundsMax) * FixMath.F64.Half;
            var size = inParams.MapBoundsMax - inParams.MapBoundsMin;
            var div = inParams.MapCellDivs;
            AllocPD(center, size, div);

            // initialize physics world
            var Damping = FixMath.F64.FromFloat(1.0f).ToF64();
            _physicsWorld = new Volatile.VoltWorld(0, Damping);
            _physicsWorld.DeltaTime = PHYSICS_FRAME_DELTATIME.ToF64();
            _physicsWorld.IterationCount = 5;

            _physicsWorld.SetContactListener(this);

            // initialize logic map
            _map = new Map();
            _map.SetMap(inParams.MapBoundsMin.X, inParams.MapBoundsMin.Z, size.X, size.Z);

            initializeParams = inParams;
            _initialized = true;

            // 打开录像
            if (!_isReplayControlMode && null == _recorder && inParams.IsOpenRecord)
            {
                StartRecord(inParams.RecordRootDir);
            }

            return true;
        }

        public bool UnInitialize()
        {
            if (_isReplayControlMode)
                return false;

            if (!_initialized) return true;

            var PendingDeleteEntities = new List<ICrowdEntityActor>(Entities);
            for (var i = 0; i < PendingDeleteEntities.Count; ++i)
            {
                DeleteEntity(PendingDeleteEntities[i].ID);
            }

            Entities.Clear();
            EntityId2Index.Clear();

            _physicsWorld.SetContactListener(null);
            _physicsWorld = null;

            _initialized = false;
            return true;
        }

        public void FrameBegin()
        {
            if (_isReplayControlMode) 
                return;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationFrameBegin());
            }
        }

        public void FrameEnd()
        {
            if (_isReplayControlMode)
                return;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationFrameBegin());
            }
        }

        #region Recorder
        public bool IsRecording()
        {
            return null != _recorder && _recorder.IsRecording;
        }
        public bool StartRecord(string outputDir)
        {
            if (!_initialized) return false;
            StopRecord();
            StopReplay();
            _recorder = new Recorder(this);
            if (_recorder.StartRecord(outputDir))
            {
                _recorder.AddReplayOperation(new OperatioMapInitial(initializeParams));
                return true;
            }
            return false;
        }
        public bool StopRecord()
        {
            if (null != _recorder)
            {
                _recorder.StopRecord();
                _recorder = null;
            }
            return false;
        }

        public bool IsReplaying()
        {
            return null != _recorder && _recorder.IsReplaying;
        }
        public bool StartReplay(string inputFile)
        {
            StopRecord();
            StopReplay();
            _recorder = new Recorder(this);
            if (_recorder.StartReplay(inputFile))
            {
                _isReplayControlMode = true;
                return true;
            }
            return false;
        }

        public bool StopReplay()
        {
            if (null != _recorder)
            {
                _recorder.StopReplay();
                _recorder = null;
            }
            _isReplayControlMode = false;
            return false;
        }

        public bool IsPauseReplay 
        { 
            get
            {
                if (null != _recorder && _recorder.IsReplaying)
                {
                    return _recorder.IsPauseReplay;
                }
                return false;
            }

            set 
            {
                if (null != _recorder && _recorder.IsReplaying)
                {
                    _recorder.IsPauseReplay = value;
                }
            } 
        }
        public FixMath.F64 ReplaySpeed 
        { 
            get
            {
                return _recorder != null? _recorder.ReplaySpeed : FixMath.F64.Zero;
            }
            
            set
            { 
                if (_recorder != null)
                {
                    _recorder.ReplaySpeed = value;
                }
            }
        }

        public void TickReplay(FixMath.F64 inDelteTime)
        {
            if (!_isReplayControlMode)
                return;
            if (null == _recorder || !_recorder.IsReplaying)
                return;

            // 临时关闭_isReplayControlMode,让其能够执行逻辑
            _isReplayControlMode = false;
            _recorder.TickReplay(inDelteTime);
            _isReplayControlMode = true;
        }
        #endregion

        public void Tick(FixMath.F64 inDelteTime)
        {
            if (_isReplayControlMode)
                return;

            if (null != _recorder && _recorder.IsRecording)
            {
                _recorder.AddReplayOperation(new OperationTick(inDelteTime));
            }

            _tickElapsedTime += inDelteTime;

            while (_tickElapsedTime > TICK_FRAME_DELTATIME)
            {
                _tickElapsedTime -= TICK_FRAME_DELTATIME;

                TickInteral(TICK_FRAME_DELTATIME);
            }
        }

        void TickInteral(FixMath.F64 inDelteTime)
        {
            ++FrameNo;

            // update movement
            for (var i = 0; i < Entities.Count; i++)
            {
                Entities[i].OnUpdate(inDelteTime);
            }

            // update physics
            if (_physicsWorld != null)
            {
                for (var step = 0; step < PHYSICS_SUBSTEP_NUM; ++step)
                {
                    for (var i = 0; i < Entities.Count; i++)
                    {
                        Entities[i].OnPrePhysics();
                    }

                    _physicsWorld.Update();

                    for (var i = 0; i < Entities.Count; i++)
                    {
                        Entities[i].OnPostPhysics();
                    }
                }
            }
            else
            {
                // resolve collision
                int MaxIterNum = 4;
                for (int iter = 0; iter < MaxIterNum; ++iter)
                {
                    for (int i = 0; i < Entities.Count; ++i)
                    {
                        if (Entities[i] is MovableEntity entity)
                        {
                            if (!entity.HasEntityState(MovableEntity.eEntityState.Moving))
                                continue;

                            entity.Displacement = entity.ResolveCollisionWithNeighbors();
                        }
                    }

                    for (int i = 0; i < Entities.Count; ++i)
                    {
                        if (Entities[i] is MovableEntity entity)
                        {
                            if (!entity.HasEntityState(MovableEntity.eEntityState.Moving))
                                continue;

                            entity.Position += entity.Displacement;
                        }
                    }
                }
            }
        }

        public void ForEachEntity(System.Action<ICrowdEntityActor> InAction)
        {
            if (null == Entities)
                return;
            
            for (var i = 0; i < Entities.Count; i++)
            {
                InAction(Entities[i]);
            }
        }

        public void ForEachContactPair(System.Action<UniqueId, UniqueId> InAction)
        {
            for (var i = 0; i < ContactPairs.Count; ++i)
            {
                InAction(ContactPairs[i].Item1, ContactPairs[i].Item2);
            }
        }
    }
}
