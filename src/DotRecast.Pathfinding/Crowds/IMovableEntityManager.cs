using System;
using System.Collections.Generic;
using System.Text;
using DotRecast.Pathfinding.Util;
using SharpSteer2;

namespace DotRecast.Pathfinding.Crowds
{
    public class CreateEntityParams
    {
        public UniqueId EntityId = UniqueId.InvalidID;
        public UnityEngine.Vector3 SpawnPosition = UnityEngine.Vector3.zero;
        public UnityEngine.Quaternion SpawnRotation = UnityEngine.Quaternion.identity;

        public IPathwayQuerier PathwayQuerier = null;
        public ILocalBoundaryQuerier LocalBoundaryQuerier = null;
        public IAnnotationService AnnotationService = null;
    };

    public interface IMovableEntityManager
    {
        int FrameNo { get; set; }

        // init and uninit function
        bool Initialize();
        bool UnInitialize();

        // create new entity
        UniqueId CreateEntity(CreateEntityParams inParams);

        // remove entity
        bool DeleteEntity(UniqueId inEntityId);

        // find entity
        MovableEntity GetEntityById(UniqueId inEntityId);
        MovableEntity[] GetEntitiesInCircle(UnityEngine.Vector3 inCenter, float inRadius);

        // tick
        void Tick(float inDelteTime);
    }
}
