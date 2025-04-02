
using System.IO;
using System;
using Pathfinding.Util;

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
            }
            catch (Exception e)
            {
                return false;
            }
            return true;
        }

        public bool StopReplay()
        {
            if (null != RecordReader)
                RecordReader.Close();
            if (null != _recordStream)
                _recordStream.Close();
            return true;
        }

        public void SerilaizeOperation(IRecordOperation operation)
        {
            var op = (int)operation.Operation;
            op.Serialize(this);
            operation.Serialize(this);
        }

        public IRecordOperation DeserializeOperation(eRecordOperation type)
        {
            return null;
        }
    }
}
