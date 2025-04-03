
using System;
using Pathfinding.Util;

namespace Pathfinding.Crowds
{
    public struct FRecordHeader
    {
        public int Version;
        public long DateTime;
    }

    public enum eEntityTemplateType
    {
        Movable = 0,
        UnMovable,
    }

    public static class DataSerialize
    {
        public static void Serialize(ref this int i, Recorder recorder)
        {
            if (recorder.IsRecording)
            {
                recorder.RecordWriter.Write(i);
            }
            else if (recorder.IsReplaying)
            {
                i = recorder.RecordReader.ReadInt32();
            }
        }

        public static void Serialize(ref this FixMath.F64 f, Recorder recorder)
        {
            if (recorder.IsRecording)
            {
                recorder.RecordWriter.Write(f.Raw);
            }
            else if (recorder.IsReplaying)
            {
                var raw = recorder.RecordReader.ReadInt64();
                f = FixMath.F64.FromRaw(raw);
            }
        }

        public static void Serialize(ref this FixMath.F64Vec2 v, Recorder recorder)
        {
            var x = v.X;
            Serialize(ref x, recorder);
            v.X = x;
            var y = v.Y;
            Serialize(ref y, recorder);
            v.Y = y;
        }

        public static void Serialize(ref this FixMath.F64Vec3 v, Recorder recorder)
        {
            var x = v.X;
            Serialize(ref x, recorder);
            v.X = x;
            var y = v.Y;
            Serialize(ref y, recorder);
            v.Y = y;
            var z = v.Z;
            Serialize(ref z, recorder);
            v.Z = z;
        }

        public static void Serialize(ref this FixMath.F64Quat q, Recorder recorder)
        {
            var x = q.X;
            Serialize(ref x, recorder);
            q.X = x;
            var y = q.Y;
            Serialize(ref y, recorder);
            q.Y = y;
            var z = q.Z;
            Serialize(ref z, recorder);
            q.Z = z;
            var w = q.W;
            Serialize(ref w, recorder);
            q.W = w;
        }

        public static void Serialize(ref this UniqueId id, Recorder recorder)
        {
            if (recorder.IsRecording)
            {
                recorder.RecordWriter.Write(id.Id);
            }
            else if (recorder.IsReplaying)
            {
                id = new UniqueId(recorder.RecordReader.ReadUInt32());
            }
        }

        public static void Serialize(ref this FRecordHeader header, Recorder recorder)
        {
            if (recorder.IsRecording)
            {
                recorder.RecordWriter.Write(header.Version);
                recorder.RecordWriter.Write(header.DateTime);
            }
            else if (recorder.IsReplaying)
            {
                header.Version = recorder.RecordReader.ReadInt32();
                header.DateTime = recorder.RecordReader.ReadInt64();
            }
        }

        public static void Serialize(ref this IMovableEntityManager.FInitializeParams param, Recorder recorder)
        {
            param.MapBoundsMin.Serialize(recorder);
            param.MapBoundsMax.Serialize(recorder);
            param.MapCellDivs.Serialize(recorder);
        }
        public static void Serialize(TMovableEntityTemplate template, Recorder recorder)
        {
            template.Density.Serialize(recorder);
            template.StopMoveRadius.Serialize(recorder);
            template.Radius.Serialize(recorder);
            template.MaxSpeed.Serialize(recorder);
            template.MaxForce.Serialize(recorder);
            template.ForwardMoveWeight.Serialize(recorder);
            template.FollowPathAheadTime.Serialize(recorder);
            template.FollowPathWeight.Serialize(recorder);
            template.AvoidObstacleAheadTime.Serialize(recorder);
            template.AvoidObstacleWeight.Serialize(recorder);
            template.PredictionAvoidIdleNeighborTime.Serialize(recorder);
            template.AvoidNeighborAheadTime.Serialize(recorder);
            template.AvoidNeighborWeight.Serialize(recorder);
            template.SeparationRadius.Serialize(recorder);
            template.SeparationAngle.Serialize(recorder);
            template.SeparationWeight.Serialize(recorder);
            template.AlignmentRadius.Serialize(recorder);
            template.AlignmentAngle.Serialize(recorder);
            template.AlignmentWeight.Serialize(recorder);
            template.CohesionRadius.Serialize(recorder);
            template.CohesionAngle.Serialize(recorder);
        }

        public static void Serialize(TUnMovableEntityTemplate template, Recorder recorder)
        {
            template.Density.Serialize(recorder);
            template.HalfExtent.Serialize(recorder);
        }

        public static void Serialize(ref CreateEntityParams create, Recorder recorder)
        {
            create.EntityId.Serialize(recorder);
            create.SpawnPosition.Serialize(recorder);
            create.SpawnRotation.Serialize(recorder);

            if (recorder.IsRecording)
            {
                if (create.Template is TMovableEntityTemplate movable)
                {
                    recorder.RecordWriter.Write((byte)eEntityTemplateType.Movable);
                    Serialize(movable, recorder);
                }
                else if (create.Template is TUnMovableEntityTemplate unmovable)
                {
                    recorder.RecordWriter.Write((byte)eEntityTemplateType.UnMovable);
                    Serialize(unmovable, recorder);
                }
            }
            else if (recorder.IsReplaying)
            {
                var type = (eEntityTemplateType)recorder.RecordReader.ReadByte();
                if (type == eEntityTemplateType.Movable)
                {
                    var movable = new TMovableEntityTemplate();
                    Serialize(movable, recorder);
                    create.Template = movable;
                }
                else if (type == eEntityTemplateType.UnMovable)
                {
                    var unmovable = new TUnMovableEntityTemplate();
                    Serialize(unmovable, recorder);
                    create.Template = unmovable;
                }
            }
        }
    }

    public enum eRecordOperation
    {
        None = 0,
        MapInitial,
        FrameBegin,
        FrameEnd,
        CreateEntity,
        DeleteEntity,
        MoveEntity,
        Tick,
    }

    public interface IRecordOperation
    {
        eRecordOperation Operation { get; }

        void Serialize(Recorder recorder);

        double ExecuteTime(Recorder recorder);

        void Execute(Recorder recorder);
    }

    public class OperatioMapInitial : IRecordOperation
    {
        IMovableEntityManager.FInitializeParams _param;

        public eRecordOperation Operation => eRecordOperation.MapInitial;

        public OperatioMapInitial(IMovableEntityManager.FInitializeParams? param = null)
        {
            _param = param?? new IMovableEntityManager.FInitializeParams();
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.UnInitialize();
                recorder.EntityManager.Initialize(_param);
            }
        }

        public void Serialize(Recorder recorder)
        {
            _param.Serialize(recorder);
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationFrameBegin : IRecordOperation
    {
        public eRecordOperation Operation => eRecordOperation.FrameBegin;

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.FrameBegin();
            }
        }

        public void Serialize(Recorder recorder)
        {
            
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationFrameEnd : IRecordOperation
    {
        public eRecordOperation Operation => eRecordOperation.FrameEnd;

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.FrameEnd();
            }
        }

        public void Serialize(Recorder recorder)
        {

        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationCreateEntity : IRecordOperation
    {
        CreateEntityParams _param;

        public OperationCreateEntity(CreateEntityParams? param = null)
        {
            _param = param?? new CreateEntityParams();
        }

        public eRecordOperation Operation => eRecordOperation.CreateEntity;

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.CreateEntity(_param);
            }
        }

        public void Serialize(Recorder recorder)
        {
            DataSerialize.Serialize(ref _param, recorder);
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationDeleteEntity : IRecordOperation
    {
        UniqueId _entityId;
        public eRecordOperation Operation => eRecordOperation.DeleteEntity;

        public OperationDeleteEntity(UniqueId? entityId = null)
        {
            _entityId = entityId?? UniqueId.InvalidID;
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.DeleteEntity(_entityId);
            }
        }

        public void Serialize(Recorder recorder)
        {
            _entityId.Serialize(recorder);
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationMoveEntity : IRecordOperation
    {
        static FixMath.F64Vec3 INVALID_TARGET = new FixMath.F64Vec3(FixMath.F64.MaxValue, FixMath.F64.MaxValue, FixMath.F64.MaxValue);

        UniqueId _entityId;
        FixMath.F64Vec3? _target;
        public eRecordOperation Operation => eRecordOperation.MoveEntity;

        public OperationMoveEntity(UniqueId? entityId = null, FixMath.F64Vec3? target = null)
        {
            _entityId = entityId ?? UniqueId.InvalidID;
            _target = target;
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying) 
            {
                recorder.EntityManager.MoveEntity(_entityId, _target);
            }
        }

        public void Serialize(Recorder recorder)
        {
            // _enetityId
            _entityId.Serialize(recorder);
            // _target
            if (recorder.IsRecording)
            {
                var p = _target?? INVALID_TARGET;
                p.Serialize(recorder);
            }
            else
            {
                var p = FixMath.F64Vec3.Zero;
                p.Serialize(recorder);
                if (p == INVALID_TARGET)
                    _target = null;
                else
                    _target = p;
            }
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }

    public class OperationTick : IRecordOperation
    {
        FixMath.F64 _deltaTime;
        public eRecordOperation Operation => eRecordOperation.Tick;

        public OperationTick(FixMath.F64? deltaTime = null)
        {
            _deltaTime = deltaTime?? FixMath.F64.Zero;
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.Tick(_deltaTime);
            }
        }

        public void Serialize(Recorder recorder)
        {
            _deltaTime.Serialize(recorder);
        }

        public double ExecuteTime(Recorder recorder)
        {
            return _deltaTime.Double;
        }
    }
}
