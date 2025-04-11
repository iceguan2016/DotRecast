
using System.IO;
using System;
using Pathfinding.Util;
using System.Collections.Generic;

namespace Pathfinding.Crowds
{
    // 录像模块，方便回放定位一些偶现问题
    public class RecorderDebugParams
    {
        // 单步添加障碍物
        public bool isStepCreateEntity = true;
        // 当前添加EntityId
        public UniqueId curEntityId = UniqueId.InvalidID;
        // 当前添加Entity数量
        public int addEntityNum = 0;
        // 调试Entity索引
        public int watchEntityIndex = -1;

        public bool isWatchNextEntity()
        {
            return watchEntityIndex > 0 && watchEntityIndex == addEntityNum;
        }

        public void reset()
        {
            curEntityId = UniqueId.InvalidID;
            addEntityNum = 0;
        }
    }

    public class Recorder
    {
        public enum eWorkMode
        {
            None = 0,
            Record,
            Replay,
        }

        static int VERSION_NO = 2;

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
        public int CurrReplayFrame { get; private set; }
        public int MaxReplayFrame { get; private set; }
        public int ReplayTargetFrame { get; set; }

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
            if (!IsRecording) return false;
            if (null != _replayOperations)
                WriteReplayOperations(_replayOperations.Count);

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

                var operation = IRecordOperation.DeserializeOperation((eRecordOperation)op, this);
                if (null == operation) break;
                // 统计帧数
                if (operation is OperationTick)
                { 
                    ++MaxReplayFrame;
                }
                _replayOperations.Enqueue(operation);
            }
        }

        void WriteReplayOperations(int count)
        {
            if (!IsRecording) return;
            int i = 0;
            while (_replayOperations.Count > 0 && i < count)
            {
                IRecordOperation.SerilaizeOperation(_replayOperations.Dequeue(), this);
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

                _recordStream = File.OpenRead(inputFile);
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
                ReplayTargetFrame = -1;
                CurrReplayFrame = 0;
                MaxReplayFrame = 0;
                IsPauseReplay = false;
                _tickElapsedTime = 0.0;

                Debug.recorderDebugParams.reset();
            }
            catch (Exception e)
            {
                return false;
            }
            return true;
        }

        public bool StopReplay()
        {
            if (!IsReplaying) return false;
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

                var stopLoop = false;
                while (!stopLoop && _tickElapsedTime > 0)
                {
                    if (Debug.locatePosition.isError)
                    {
                        IsPauseReplay = true;
                        stopLoop = true;
                    }

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

                        if (operation is OperationTick)
                        {
                            ++CurrReplayFrame;
                            // 检查目标帧号
                            if (ReplayTargetFrame > 0 && CurrReplayFrame == ReplayTargetFrame)
                            {
                                IsPauseReplay = true;
                                stopLoop = true;
                            }
                        }

                        // 单步调试添加Entity
                        if (operation is OperationCreateEntity)
                        {
                            Debug.recorderDebugParams.addEntityNum++;
                            if (Debug.recorderDebugParams.isStepCreateEntity)
                            {
                                IsPauseReplay = true;
                                stopLoop = true;
                            }
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }
}
