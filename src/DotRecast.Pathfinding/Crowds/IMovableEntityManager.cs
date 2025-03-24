using Pathfinding.Util;
using SharpSteer2;

namespace Pathfinding.Crowds
{
    public class CreateEntityParams
    {
        public UniqueId EntityId = UniqueId.InvalidID;
        public FixMath.F64Vec3 SpawnPosition = FixMath.F64Vec3.Zero;
        public FixMath.F64Quat SpawnRotation = FixMath.F64Quat.Identity;

        public TEntityTemplate Template = null;
        public IPathwayQuerier PathwayQuerier = null;
        public ILocalBoundaryQuerier LocalBoundaryQuerier = null;
        public IAnnotationService AnnotationService = null;
    };

    public interface IMovableEntityManager
    {
        int FrameNo { get; set; }

        Volatile.VoltWorld PhysicsWorld { get; }

        // init and uninit function
        bool Initialize();
        bool UnInitialize();

        // create new entity
        UniqueId CreateEntity(CreateEntityParams inParams);

        // remove entity
        bool DeleteEntity(UniqueId inEntityId);

        // find entity
        ICrowdEntityActor GetEntityById(UniqueId inEntityId);
        ICrowdEntityActor[] GetEntitiesInCircle(FixMath.F64Vec3 inCenter, FixMath.F64 inRadius);

        // entity iteration
        void ForEachEntity(System.Action<ICrowdEntityActor> InAction);

        // tick
        void Tick(FixMath.F64 inDelteTime);
    }
}
