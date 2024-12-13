using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Security.Cryptography;
using System.Security.Principal;
using DotRecast.Pathfinding.Util;
using SharpSteer2;
using SharpSteer2.Database;
using SharpSteer2.Helpers;

namespace DotRecast.Pathfinding.Crowds
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
            Vector3 center = Vector3.Zero;
            const float DIV = 10.0f;
            Vector3 divisions = new Vector3(DIV, DIV, DIV);
            const float DIAMETER = WORLD_RADIUS * 1.1f * 2;
            Vector3 dimensions = new Vector3(DIAMETER, DIAMETER, DIAMETER);
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

            // position
            entity.Position = Vector3Helpers.FromUntiyVector(inParams.SpawnPosition);
            // rotation
            LocalSpaceBasisHelpers.FromUnityRotation(inParams.SpawnRotation, out var forward, out var side, out var up);
            entity.Forward = forward;
            entity.Side = side;
            entity.Up = up;

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
        public MovableEntity[] GetEntitiesInCircle(UnityEngine.Vector3 inCenter, float inRadius)
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

        public void Tick(float inDelteTime)
        {
            ++FrameNo;
        }
    }
}
