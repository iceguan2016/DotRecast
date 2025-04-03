
using System.IO;
using System;
using Pathfinding.Util;
using System.Collections.Generic;

namespace Pathfinding.Crowds
{
    // 录像模块，方便回放定位一些偶现问题
    public class Recorder
    {
        public enum eWorkMode
        {
            None = 0,
            Record,
            Replay,
        }

        static int VERSION_NO = 1;

        public eWorkMode WorkMode { get; private set; }
        public IMovableEntityManager EntityManager { get; private set; }

        private FileStream _recordStream = null;
        // record 
        public BinaryWriter RecordWriter { get; private set; }
        public bool IsRecording { get { return WorkMode == eWorkMode.Record && null != RecordWriter; } }

        // replay
        public BinaryReader RecordReader { get; private set; }
        public bool IsReplaying { get { return WorkMode == eWorkMode.Replay && null != RecordReader; } }
        public bool IsPauseReplay { get; set; }
        public FixMath.F64 ReplaySpeed { get; set; }

        double _tickElapsedTime = 0.0;

        static int CACHE_REPLAY_OPERATION_COUNT = 100;
        Queue<IRecordOperation> _replayOperations = null;

        public Recorder(IMovableEntityManager entityManager)
        {
            EntityManager = entityManager;
        }

        public bool StartRecord(string outputDir)
        {
            try
            {
                StopRecord();

                // 1.生成录像文件
                if (!Directory.Exists(outputDir))
                {
                    Directory.CreateDirectory(outputDir);
                }

                var dateTime = DateTime.Now;
                var filePath = outputDir + "/Record" + dateTime.ToString("yyyy_MM_dd_HH_mm_ss") + ".record";
                _recordStream = File.Open(filePath, FileMode.OpenOrCreate);
                RecordWriter = new BinaryWriter(_recordStream);

                WorkMode = eWorkMode.Record;

                // 2.写入信息头
                FRecordHeader header = new FRecordHeader()
                {
                    Version = VERSION_NO,
                    DateTime = dateTime.Ticks,
                };

                header.Serialize(this);

                // 缓存一定数量的Operations再写入文件
                _replayOperations = new Queue<IRecordOperation>(CACHE_REPLAY_OPERATION_COUNT);
            }
            catch (Exception e)
            {
                return false;
            }
            return true;
        }

        public bool StopRecord()
        {
            if (null != RecordWriter)
                RecordWriter.Close();
            if (null != _recordStream)
                _recordStream.Close();

            RecordWriter = null;
            _recordStream = null;
            return false;
        }

        void ReadReplayOperations(int count)
        { 
            if (!IsReplaying) return;

            while (_recordStream.Position < _recordStream.Length 
                && _replayOperations.Count < count) 
            {
                int op = 0;
                op.Serialize(this);

                var operation = DeserializeOperation((eRecordOperation)op);
                if (null == operation) break;
                _replayOperations.Enqueue(operation);
            }
        }

        void WriteReplayOperations(int count)
        {
            if (!IsRecording) return;
            int i = 0;
            while (_replayOperations.Count > 0 && i < count)
            {
                SerilaizeOperation(_replayOperations.Dequeue());
                ++i;
            }
        }

        public bool StartReplay(string inputFile)
        {
            try
            {
                StopReplay();

                // 1.读取录像文件
                if (!File.Exists(inputFile))
                {
                    return false;
                }

                _recordStream = File.Open(inputFile, FileMode.Open);
                RecordReader = new BinaryReader(_recordStream);

                WorkMode = eWorkMode.Replay;

                // 2.读取信息头
                var header = new FRecordHeader();
                header.Serialize(this);
                if (header.Version != VERSION_NO)
                {
                    StopRecord();
                    return false;
                }

                // 缓存一定数量的Operations再播放
                _replayOperations = new Queue<IRecordOperation>(CACHE_REPLAY_OPERATION_COUNT);
                ReadReplayOperations(CACHE_REPLAY_OPERATION_COUNT);

                // 设置基本参数
                ReplaySpeed = FixMath.F64.One;
                IsPauseReplay = false;
                _tickElapsedTime = 0.0;
            }
            catch (Exception e)
            {
                return false;
            }
            return true;
        }

        public bool StopReplay()
        {
            if (null != _replayOperations)
                WriteReplayOperations(_replayOperations.Count);

            if (null != RecordReader)
                RecordReader.Close();
            if (null != _recordStream)
                _recordStream.Close();
            return true;
        }

        public void AddReplayOperation(IRecordOperation operation)
        {
            WriteReplayOperations(CACHE_REPLAY_OPERATION_COUNT);

            _replayOperations.Enqueue(operation);
        }

        public void TickReplay(FixMath.F64 inDeltaTime)
        {
            if (IsReplaying && !IsPauseReplay)
            {
                _tickElapsedTime += inDeltaTime.Double * ReplaySpeed.Double;

                while (_tickElapsedTime > 0)
                {
                    ReadReplayOperations(CACHE_REPLAY_OPERATION_COUNT);

                    if (_replayOperations.Count <= 0)
                        return;
                    var operation = _replayOperations.Peek();
                    if (operation == null)
                        return;

                    var executeTime = operation.ExecuteTime(this);
                    if (_tickElapsedTime >= executeTime)
                    {
                        _tickElapsedTime -= executeTime;
                        _replayOperations.Dequeue().Execute(this);
                    }
                }
            }
        }

        void SerilaizeOperation(IRecordOperation operation)
        {
            var op = (int)operation.Operation;
            op.Serialize(this);
            operation.Serialize(this);
        }

        IRecordOperation DeserializeOperation(eRecordOperation type)
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
                case eRecordOperation.Tick:
                    operation = new OperationTick();
                    break;
                default:
                    break;

            }
            operation.Serialize(this);
            return operation;
        }
    }
}
