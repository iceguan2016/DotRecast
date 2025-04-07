
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

        public static void Serialize(ref this int? i, Recorder recorder)
        {
            if (recorder.IsRecording)
            {
                var v = i ?? int.MaxValue;
                recorder.RecordWriter.Write(v);
            }
            else if (recorder.IsReplaying)
            {
                var i32 = recorder.RecordReader.ReadInt32();
                if (i32 == int.MaxValue)
                    i = null;
                else
                    i = i32;
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

        public static void Serialize(ref this FixMath.F64? f, Recorder recorder)
        {
            if (recorder.IsRecording) 
            {
                var v = f ?? FixMath.F64.MaxValue;
                recorder.RecordWriter.Write(v.Raw);
            }
            else if (recorder.IsReplaying)
            {
                var f64 = FixMath.F64.FromRaw(recorder.RecordReader.ReadInt64());
                if (f64 == FixMath.F64.MaxValue)
                    f = null;
                else
                    f = f64;
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
            create.TemplateId.Serialize(recorder);
            create.SpawnPosition.Serialize(recorder);
            create.SpawnRotation.Serialize(recorder);
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
        SetEntityParams,
        Tick,
        RegisterTemplate,
    }

    public interface IRecordOperation
    {
        eRecordOperation Operation { get; }

        void Serialize(Recorder recorder);

        double ExecuteTime(Recorder recorder);

        void Execute(Recorder recorder);

        public static void SerilaizeOperation(IRecordOperation operation, Recorder recorder)
        {
            var op = (int)operation.Operation;
            op.Serialize(recorder);
            operation.Serialize(recorder);
        }
        public static IRecordOperation DeserializeOperation(eRecordOperation type, Recorder recorder)
        {
            IRecordOperation operation = null;
            switch (type)
            {
                case eRecordOperation.MapInitial:
                    operation = new OperatioMapInitial();
                    break;
                case eRecordOperation.FrameBegin:
                    operation = new OperationFrameBegin();
                    break;
                case eRecordOperation.FrameEnd:
                    operation = new OperationFrameEnd();
                    break;
                case eRecordOperation.CreateEntity:
                    operation = new OperationCreateEntity();
                    break;
                case eRecordOperation.DeleteEntity:
                    operation = new OperationDeleteEntity();
                    break;
                case eRecordOperation.MoveEntity:
                    operation = new OperationMoveEntity();
                    break;
                case eRecordOperation.SetEntityParams:
                    operation = new OperationSetEntityParams();
                    break;
                case eRecordOperation.Tick:
                    operation = new OperationTick();
                    break;
                default:
                    break;

            }
            operation.Serialize(recorder);
            return operation;
        }
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

    public class OperationSetEntityParams : IRecordOperation
    {
        UniqueId _entityId;
        FixMath.F64? _radius;
        FixMath.F64? _maxSpeed;
        FixMath.F64? _maxForce;
        int? _groupMask;
        int? _groupToAvoid;

        public eRecordOperation Operation => eRecordOperation.SetEntityParams;

        public OperationSetEntityParams(
            UniqueId? entityId = null, 
            FixMath.F64? radius = null, 
            FixMath.F64? maxSpeed = null, 
            FixMath.F64? maxForce = null,
            int? groupMask = null,
            int? groupToAvoid = null)
        {
            _entityId = entityId ?? UniqueId.InvalidID;
            _radius = radius;
            _maxSpeed = maxSpeed;
            _maxForce = maxForce;
            _groupMask = groupMask;
            _groupToAvoid = groupToAvoid;
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying)
            {
                recorder.EntityManager.SetEntityParams(_entityId, _radius, _maxSpeed, _maxForce, _groupMask, _groupToAvoid);
            }
        }

        public void Serialize(Recorder recorder)
        {
            _entityId.Serialize(recorder);
            _radius.Serialize(recorder);
            _maxSpeed.Serialize(recorder);
            _maxForce.Serialize(recorder);
            _groupMask.Serialize(recorder);
            _groupToAvoid.Serialize(recorder);
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

    public class OperationRegisterTemplate : IRecordOperation
    {
        UniqueId _tid;
        TEntityTemplate _template;
        public eRecordOperation Operation => eRecordOperation.RegisterTemplate;

        public OperationRegisterTemplate(UniqueId? tid = null, TEntityTemplate? template = null)
        {
            _tid = tid?? UniqueId.InvalidID;
            _template = template?? null;
        }

        public void Execute(Recorder recorder)
        {
            if (recorder.IsReplaying && _tid != UniqueId.InvalidID && _template != null)
            {
                recorder.EntityManager.RegisterTemplate(_tid, _template);
            }
        }

        public void Serialize(Recorder recorder)
        {
            _tid.Serialize(recorder);
            if (recorder.IsRecording)
            {
                if (null != _template)
                {
                    if (_template is TMovableEntityTemplate movable)
                    {
                        var flag = 1;
                        flag.Serialize(recorder);
                        DataSerialize.Serialize(movable, recorder);
                    }
                    else if (_template is TUnMovableEntityTemplate unmovable)
                    {
                        var flag = 2;
                        flag.Serialize(recorder);
                        DataSerialize.Serialize(unmovable, recorder);
                    }
                    else
                    {
                        var flag = 3;
                        flag.Serialize(recorder);
                    }
                }
                else
                {
                    var flag = 0;
                    flag.Serialize(recorder);
                }
            }
            else if (recorder.IsReplaying)
            {
                var flag = 0;
                flag.Serialize(recorder);

                if (flag == 1)
                {
                    var template = new TMovableEntityTemplate();
                    DataSerialize.Serialize(template, recorder);
                    _template = template;
                }
                else if (flag == 2)
                {
                    var template = new TUnMovableEntityTemplate();
                    DataSerialize.Serialize(template, recorder);
                    _template = template;
                }
            }
        }

        public double ExecuteTime(Recorder recorder)
        {
            return 0.0;
        }
    }
}
