using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using SharpSteer2.Obstacles;

namespace DotRecast.Pathfinding.Crowds
{
    public class FCycleDataBuffer<T> where T : struct
    {
        T[]     _data = null;
        int[]   _frameNos = null;
        int     _buffSize = 64;
        int     _dataSize = 0;

        public FCycleDataBuffer(int inBuffSize)
        {
            _buffSize = UnityEngine.Mathf.NextPowerOfTwo(inBuffSize);
            _data = new T[_buffSize];
            Array.Fill(_data, new T());

            _frameNos= new int[_buffSize];
            _dataSize = 0;
        }

        public void Add(int inFrameNo, T inElement)
		{
			int index = _dataSize & (_buffSize - 1);
            _data[index] = inElement;
			_frameNos[index] = inFrameNo;
			++_dataSize;
		}

        public ref T Alloc(int inFrameNo)
        {
            int index = _dataSize & (_buffSize - 1);
            _frameNos[index] = inFrameNo;
            ++_dataSize;
            return ref _data[index];
        }

        public void ForEach(System.Action<int, T> inFunc)
		{
			for (int i = Math.Max(_dataSize - _buffSize, 0); i < _dataSize; ++i)
			{
				int index = i & (_buffSize - 1);
                inFunc(_frameNos[index], _data[index]);
            }
        }

        public T? FindData(int inFrameNo)
		{
            for (int i = Math.Max(_dataSize - _buffSize, 0); i < _dataSize; ++i)
            {
                int index = i & (_buffSize - 1);
                if (_frameNos[index] == inFrameNo)
				{
					return _data[index];
				}
            }
            return null;
        }

        public int FindNearestPrevFrame(int inFrameNo)
        {
            for (int i = _dataSize - 1; i >= Math.Max(_dataSize - _buffSize, 0); --i)
            {
                int index = i & (_buffSize - 1);
                int frameNo = _frameNos[index];
                if (frameNo < inFrameNo)
                {
                    return frameNo;
                }
            }
            return inFrameNo;
        }

        public int FindNearestNextFrame(int inFrameNo)
        {
            for (int i = Math.Max(_dataSize - _buffSize, 0); i < _dataSize; ++i)
            {
                int index = i & (_buffSize - 1);
                int frameNo = _frameNos[index];
                if (frameNo > inFrameNo)
                {
                    return frameNo;
                }
            }
            return inFrameNo;
        }
    }

    public class MovableEntityDebuger
    {
        // 缓存历史intersection信息
        public FCycleDataBuffer<PathIntersection> PathInterSectionBuff = new FCycleDataBuffer<SharpSteer2.Obstacles.PathIntersection>(256);

        // 缓存历史entity的信息
        public struct EntityDebugInfo
        {
            public float maxSpeed;
            public float radius;
            public Vector3 position;
            public Vector3 velocity;
            public Vector3 steerPosition;

            public Vector3 forward;
            public Vector3 side;
            public Vector3 up;
        };
        public FCycleDataBuffer<EntityDebugInfo> EntityDebugInfoBuff = new FCycleDataBuffer<EntityDebugInfo>(1024);

        // Steer force
        public struct SteerForceInfo
        {
            public Vector3 position;
            public Vector3 steerForce;
            public float maxForce;
            
            public Vector3 obstacleForce;
            public Vector3 targetForce;
            public Vector3 avoidNeighborFoce;

            public void reset()
            {
                position = Vector3.Zero;
                steerForce = Vector3.Zero;
                obstacleForce = Vector3.Zero;
                targetForce = Vector3.Zero;
                avoidNeighborFoce = Vector3.Zero;
            }
        };
        public FCycleDataBuffer<SteerForceInfo> SteerFoceInfoBuff = new FCycleDataBuffer<SteerForceInfo>(1024);

        // Steer avoid neighbors
        public struct SteerAvoidNeighborInfo
        {
            public Vector3 threatPosition;
            public float threatRadius;
            public Vector3 ourFuturePosition;     // 相交位置(self)
            public Vector3 threatFuturePosition;  // 相交位置(neighbor)
            public PathIntersection pathIntersection;
        };
        public FCycleDataBuffer<SteerAvoidNeighborInfo> SteerAvoidNeighborInfoBuff = new FCycleDataBuffer<SteerAvoidNeighborInfo>(1024);
    }
}
