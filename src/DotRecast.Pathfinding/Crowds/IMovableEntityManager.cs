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

        public IAnnotationService AnnotationService = null;
    };

    public interface IMovableEntityManager
    {
        // init and uninit function
        bool Initialize();
        bool UnInitialize();

        // create new entity
        UniqueId CreateEntity(CreateEntityParams InParams);

        // remove entity
        bool DeleteEntity(UniqueId InEntityId);
    }
}
