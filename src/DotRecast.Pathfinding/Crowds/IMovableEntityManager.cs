using System.IO;
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

        Map Map { get; }

        // initialize and uninitialized function
        public struct FInitializeParams
        {
            // 地图rect范围：(min.x, min.y) - (max.x, max.y)
            public FixMath.F64Vec3 MapBoundsMin;
            public FixMath.F64Vec3 MapBoundsMax;

            // MapCellDivs变量在构建map cell使用，表示把rect区域(min.x, min.y) - (max.x, max.y)，分割成多少等分
            public int MapCellDivs;

            // 录像相关配置参数
            public bool IsOpenRecord;
            public string RecordRootDir;
        }
        bool Initialize(FInitializeParams inParams);
        bool UnInitialize();

        void FrameBegin();
        void FrameEnd();

        bool IsRecording();
        bool StartRecord(string outputDir);
        bool StopRecord();

        bool IsReplaying();
        bool StartReplay(string inputFile);
        bool StopReplay();

        // create new entity
        UniqueId CreateEntity(CreateEntityParams inParams);

        // remove entity
        bool DeleteEntity(UniqueId inEntityId);

        // 
        bool MoveEntity(UniqueId inEntityId, FixMath.F64Vec3? target);

        // find entity
        ICrowdEntityActor GetEntityById(UniqueId inEntityId);
        ICrowdEntityActor[] GetEntitiesInCircle(FixMath.F64Vec3 inCenter, FixMath.F64 inRadius);

        // entity iteration
        void ForEachEntity(System.Action<ICrowdEntityActor> InAction);
        void ForEachContactPair(System.Action<UniqueId, UniqueId> InAction);

        // tick
        void Tick(FixMath.F64 inDelteTime);
        void TickReplay(FixMath.F64 inDelteTime);
    }
}
