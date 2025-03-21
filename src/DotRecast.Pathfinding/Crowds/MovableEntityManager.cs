using System.Collections.Generic;
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
        List<MovableEntity> MovableEntities = new List<MovableEntity>();

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

        public int FrameNo { get; set; }

        public UniqueId CreateEntity(CreateEntityParams inParams)
        {
            var entity = new MovableEntity(this, inParams.PathwayQuerier, inParams.LocalBoundaryQuerier, inParams.AnnotationService);
            if (null == entity) return UniqueId.InvalidID;

            entity.ID = inParams.EntityId.IsValid()? inParams.EntityId : new UniqueId(_entityIdCounter++);

            // map EntityId and Entity
            MovableEntities.Add(entity);
            var entityIndex = MovableEntities.Count - 1;
            EntityId2Index.Add(entity.ID, entityIndex);
            // Set template
            entity.Template = inParams.Template;

            // position
            entity.Position = inParams.SpawnPosition;
            // rotation
            entity.Forward = inParams.SpawnRotation * Vector3Helpers.Forward;
            entity.Side = inParams.SpawnRotation * Vector3Helpers.Right;
            entity.Up = inParams.SpawnRotation * Vector3Helpers.Up;

            // add to pd
            entity.NewPD(_pd);
            entity.OnCreate();

            return entity.ID;
        }

        public bool DeleteEntity(UniqueId inEntityId)
        {
            if (EntityId2Index.TryGetValue(inEntityId, out var findIndex))
	        {
                var slotIndex = findIndex;
                var entity = MovableEntities[slotIndex];

                entity.NewPD(null);
                entity.OnDelete();

                if ((slotIndex + 1) < MovableEntities.Count)
                {
                    var lastSlotIndex = MovableEntities.Count - 1;
                    var lastEntity = MovableEntities[lastSlotIndex];
                    var lastEntityId = lastEntity.ID;
                    EntityId2Index[lastEntityId] = slotIndex;
                }
                Utilities.RemoveAtSwap(MovableEntities, slotIndex);
                EntityId2Index.Remove(inEntityId);
                return true;
            }
            return false;
        }

        public MovableEntity GetEntityById(UniqueId inEntityId)
        {
            if (EntityId2Index.TryGetValue(inEntityId, out var entityIndex))
            {
                return MovableEntities[entityIndex];
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
                if (neighbor is MovableEntity) entities.Add(neighbor as MovableEntity);
            }
            return entities.ToArray();
        }

        public bool Initialize()
        {
            if (_initialized) return true;

            FrameNo = 1;

            AllocPD();
            return true;
        }

        public bool UnInitialize()
        {
            if (!_initialized) return true;

            return true;
        }

        public void Tick(FixMath.F64 inDelteTime)
        {
            ++FrameNo;

            // update movement
            for ( var i = 0; i < MovableEntities.Count; i++ )
            {
                MovableEntities[i].OnUpdate(inDelteTime);
            }

            // resolve collision
            int MaxIterNum = 4;
            for (int iter = 0; iter < MaxIterNum; ++iter)
            {
                for (int i = 0; i < MovableEntities.Count; ++i)
                {
                    var entity = MovableEntities[i];
                    if (!entity.HasEntityState(MovableEntity.eEntityState.Moving))
                        continue;

                    entity.Displacement = entity.ResolveCollisionWithNeighbors();
                }

                for (int i = 0; i < MovableEntities.Count; ++i)
                {
                    var entity = MovableEntities[i];
                    if (!entity.HasEntityState(MovableEntity.eEntityState.Moving))
                        continue;

                    entity.Position += entity.Displacement;
                }
            }
        }

        public void ForEachEntity(System.Action<MovableEntity> InAction)
        {
            for (var i = 0; i < MovableEntities.Count; i++)
            {
                InAction(MovableEntities[i]);
            }
        }
    }
}
