using System.Collections.Generic;
using Pathfinding.Crowds;
using Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Database;
using SharpSteer2.Helpers;

namespace Pathfinding.Crowds
{
    public class MovableEntityManager : IMovableEntityManager
    {
        public const float WORLD_RADIUS = 50;

        // 记录单位Id到索引对象的映射
        Dictionary<UniqueId, int> EntityId2Index =  new Dictionary<UniqueId, int>();
        List<ICrowdEntityActor> Entities = new List<ICrowdEntityActor>();

        // pointer to database used to accelerate proximity queries
        private IProximityDatabase<IVehicle> _pd;

        // for purposes of demonstration, allow cycling through various
        // types of proximity databases.  this routine is called when the
        // Demo user pushes a function key.
        private void AllocPD()
        {
            // allocate new PD
            var center = FixMath.F64Vec3.Zero;
            var DIV = FixMath.F64.FromFloat(10.0f);
            var divisions = new FixMath.F64Vec3(DIV, DIV, DIV);
            var DIAMETER = FixMath.F64.FromFloat(WORLD_RADIUS * 1.1f * 2);
            var dimensions = new FixMath.F64Vec3(DIAMETER, DIAMETER, DIAMETER);
            _pd = new LocalityQueryProximityDatabase<IVehicle>(center, dimensions, divisions);
        }

        private bool _initialized = false;
        private uint _entityIdCounter = 1;

        // physics world
        private Volatile.VoltWorld _physicsWorld = null;

        public int FrameNo { get; set; }

        public Volatile.VoltWorld PhysicsWorld { get { return _physicsWorld; } }

        public UniqueId CreateEntity(CreateEntityParams inParams)
        {
            ICrowdEntityActor entityActor = null;
            if (inParams.Template is TMovableEntityTemplate)
            {
                entityActor = new MovableEntity(this, inParams.PathwayQuerier, inParams.LocalBoundaryQuerier, inParams.AnnotationService);
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

        public MovableEntity GetEntityById(UniqueId inEntityId)
        {
            if (EntityId2Index.TryGetValue(inEntityId, out var entityIndex))
            {
                return Entities[entityIndex] as MovableEntity;
            }
            return null;
        }
        public MovableEntity[] GetEntitiesInCircle(FixMath.F64Vec3 inCenter, FixMath.F64 inRadius)
        {
            var createEntityParams = new CreateEntityParams() 
            {
                SpawnPosition = inCenter,
            };
            var entityId = CreateEntity(createEntityParams);
            if (!entityId.IsValid()) return null;

            var entity = GetEntityById(entityId);
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

        public bool Initialize()
        {
            if (_initialized) return true;

            FrameNo = 1;

            AllocPD();

            // initialize physics world
            var Damping = FixMath.F64.FromFloat(1.0f).ToF64();
            _physicsWorld = new Volatile.VoltWorld(0, Damping);
            _physicsWorld.DeltaTime = FixMath.F64.FromDouble(0.05).ToF64();
            _physicsWorld.IterationCount = 5;

            return true;
        }

        public bool UnInitialize()
        {
            if (!_initialized) return true;

            _physicsWorld = null;

            return true;
        }

        public void Tick(FixMath.F64 inDelteTime)
        {
            ++FrameNo;

            // update movement
            for ( var i = 0; i < Entities.Count; i++ )
            {
                Entities[i].OnUpdate(inDelteTime);
            }

            // update physics
            if (_physicsWorld != null)
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

        public void ForEachEntity(System.Action<MovableEntity> InAction)
        {
            for (var i = 0; i < Entities.Count; i++)
            {
                if (Entities[i] is MovableEntity entity)
                {
                    InAction(entity);
                }
            }
        }
    }
}
