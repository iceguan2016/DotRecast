using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Pathfinding.Crowds;
using UnityEngine;
using static System.Net.WebRequestMethods;
using static Pathfinding.Triangulation.Data.Mesh;
using static Pathfinding.Triangulation.Math.Geom2D;

namespace Pathfinding.Util
{
    public interface IDrawInterface
    {
        float TerrainHeight { get; set; }

        Vector3 ToVec3(FixMath.F64Vec2 v);

        void DrawLine(Vector3 a, Vector3 b, Color c, float lineWidth=1.0f);

        void DrawArrow(Vector3 start, Vector3 end, Vector2 arrowSize, float lineWidth, Color c);

        void DrawCube(Vector3 p, Vector3 size, Color c);

        void DrawCircle(Vector3 p, float r, Color c);

        void DrawTriangle(Vector3 v0, Vector3 v1, Vector3 v2, Color c);

        void DrawSolidPlane(Vector3 point, Vector3 normal, Vector2 size, Color c);

        void DrawSolidCube(Vector3 p, Quaternion q, Vector3 size, Color c);
    }

    public enum eSimulationMode
    {
        Normal = 0,     // 正常模拟模式
        Playback,       // 回放模式
        Replay,         // 录像模式
    }

    public static class Debug
    {
        private static string LogRootPath = "H:/0.Gitbub/DotRecast/debug";
        private static StreamWriter LogWriter = null;
        private static int LogCount = 0;

        public static void LogToFile(string text)
        {
            if (null == LogWriter)
            {
                var Path = LogRootPath + "/" + DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss");
                LogWriter = new StreamWriter(Path);
            }

            if (null != LogWriter)
            {
                LogWriter.WriteLine(text);
                LogWriter.Flush();
            }
        }

        public static IDrawInterface drawInterface = null;

        public static void Assert(bool condition, string desc="")
        {
            System.Diagnostics.Debug.Assert(condition);
        }

        public static void Log(string content)
        { 

        }

        public static void LogWarning(string content)
        {

        }

        public static void LogError(string content)
        {

        }

#if ENABLE_NAVMESH_DEBUG
        public static RecorderDebugParams recorderDebugParams = new RecorderDebugParams();
        // 调试LocatePositionProcedure
        public static LocatePositionProcedure locatePosition = new LocatePositionProcedure();
        // 调试InsertObject
        public static InsertObjectParams insertObject = new InsertObjectParams();
        public static InsertConstraintSegmentProcedure insertConstraintSegmentProcedure = new InsertConstraintSegmentProcedure();
        public static InsertVertexProcedure insertVertex = new InsertVertexProcedure();

        public static int WatchFaceID = -1;
#endif

        // 测试单个neighbor
        public static int _drawVOIndex = 10;
        public static int WatchIndex = 0;
        public static bool IsWatchingIndex(int count, int index)
        {
            return (WatchIndex % count) == index;
        }

        public static string RecordRootDir = "H:/0.Gitbub/DotRecast/record";
        public static string ReplayFileName = "";
        public static List<FileInfo> GetLatestRecordFiles(int count)
        {
            var fileExtension = "record";
            try
            {
                // 获取所有文件并按最后修改时间降序排序
                var sortedFiles = Directory.EnumerateFiles(RecordRootDir, $"*{fileExtension}")
                                 .Select(file => new FileInfo(file))
                                 .OrderBy(file => file.LastWriteTime)
                                 .ToList();

                // 输出结果
                Debug.Log($"文件列表（按修改时间降序排序）:");
                foreach (var file in sortedFiles)
                {
                    Debug.Log($"{file.Name,-40} | 修改时间: {file.LastWriteTime}");
                }

                int num = System.Math.Min(sortedFiles.Count, count);
                return sortedFiles.Skip(0).Take(num).ToList();
            }
            catch (DirectoryNotFoundException)
            {
                Debug.LogError("目录不存在！");
            }
            catch (UnauthorizedAccessException)
            {
                Debug.LogError("没有访问权限！");
            }
            catch (Exception ex)
            {
                Debug.LogError($"发生错误: {ex.Message}");
            }
            return null;
        }

        // 播放模式
        private static eSimulationMode _simulationMode = eSimulationMode.Normal;
        public static bool IsSimulationMode(eSimulationMode InMode) { return _simulationMode == InMode; }
        public static eSimulationMode GetSimlationMode() { return _simulationMode; }    
        public static void SetSimlationMode(eSimulationMode InMode) { _simulationMode = InMode; }
        public static int PlaybackFrameNo { get; set; }
        // 暂停/恢复模拟
        private static bool _isPaused = false;
        public static void Pause() { _isPaused = true; }
        public static void Resume() { _isPaused = false; }
        public static bool IsPaused() { return _isPaused; }
        // 单步执行
        private static bool _isNextStep = false;
        public static void NextStep() { _isNextStep = true; } 
        public static bool CanNextStep()
        {
            if (IsSimulationMode(eSimulationMode.Playback))
            {
                return false;
            }

            if (!_isPaused || _isNextStep)
            {
                _isNextStep = false;
                return true;
            }
            return false;
        }
        // 当前调试EntityId
        public static UniqueId DebugEntityId { get; set; }
        // 当前Entity调试数据类型
        public static MovableEntityDebuger.eDebugerDataType DebugerDataType { get; set; }

        public static void DrawLine(Vector3 a, Vector3 b, Color c, float duration=0.0f)
        { 
            if (drawInterface != null)
            {
                drawInterface.DrawLine(a, b, c);
            }
        }

        public static void DrawRay(Vector3 origin, Vector3 direction, Color c, float duration=0.0f)
        {
            if (drawInterface != null)
            {
                var end = origin + direction * 30.0f;
                drawInterface.DrawLine(origin, end, c);
            }
        }

        public static void DrawCube(Vector3 center, Vector3 extent, Color c, float duration = 0.0f)
        {
            Vector3[] Vertices = new Vector3[] {
                center + new Vector3( extent.x, -extent.y,  extent.z), // 0
                center + new Vector3(-extent.x, -extent.y,  extent.z), // 1
                center + new Vector3(-extent.x, -extent.y, -extent.z), // 2
                center + new Vector3( extent.x, -extent.y, -extent.z), // 3

                center + new Vector3( extent.x,  extent.y,  extent.z), // 4
                center + new Vector3(-extent.x,  extent.y,  extent.z), // 5
                center + new Vector3(-extent.x,  extent.y, -extent.z), // 6
                center + new Vector3( extent.x,  extent.y, -extent.z), // 7
            };

            int[] Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
            };

            for (int i = 0; i < Indices.Length; i += 2)
            {
                Debug.DrawLine(Vertices[Indices[i]], Vertices[Indices[i + 1]], c);
            }
        }

        public static void DrawObbCube(Vector3 InPosition, Quaternion InRotation, Vector3 InHalfExtent, Color InColor)
        {
            Vector3[] Vertices = new Vector3[] {
                InPosition + InRotation * new Vector3( InHalfExtent.x, -InHalfExtent.y,  InHalfExtent.z), // 0
                InPosition + InRotation * new Vector3(-InHalfExtent.x, -InHalfExtent.y,  InHalfExtent.z), // 1
                InPosition + InRotation * new Vector3(-InHalfExtent.x, -InHalfExtent.y, -InHalfExtent.z), // 2
                InPosition + InRotation * new Vector3( InHalfExtent.x, -InHalfExtent.y, -InHalfExtent.z), // 3

                InPosition + InRotation * new Vector3( InHalfExtent.x,  InHalfExtent.y,  InHalfExtent.z), // 4
                InPosition + InRotation * new Vector3(-InHalfExtent.x,  InHalfExtent.y,  InHalfExtent.z), // 5
                InPosition + InRotation * new Vector3(-InHalfExtent.x,  InHalfExtent.y, -InHalfExtent.z), // 6
                InPosition + InRotation * new Vector3( InHalfExtent.x,  InHalfExtent.y, -InHalfExtent.z), // 7
            };

            int[] Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
            };

            for (int i = 0; i < Indices.Length; i += 2)
            {
                Debug.DrawLine(Vertices[Indices[i]], Vertices[Indices[i + 1]], InColor);
            }
        }

        public static void DrawSphere(Vector3 center, float radius, Color c, float duration = 0.0f)
        { 
        }
    }
}
