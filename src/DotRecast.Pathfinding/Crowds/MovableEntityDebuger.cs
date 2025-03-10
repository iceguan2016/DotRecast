using System;
using SharpSteer2;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;
using DotRecast.Pathfinding.Util;
using DotRecast.Detour;
using System.Threading;

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
        public enum eDebugerDataType
        {
            PathIntersection = 0,
	        LocalBoundary,
	        EntityInfo,
	        SteerForceInfo,
	        SteerAvoidNeighborInfo,
        }

        static int SteerDebugerDrawFrames = 10;

        // 缓存历史intersection信息
        public FCycleDataBuffer<PathIntersection> PathInterSectionBuff = new FCycleDataBuffer<SharpSteer2.Obstacles.PathIntersection>(256);

        public struct RectangeObstacleInfo
        {
            public FixMath.F64Vec3 Position;
            public FixMath.F64Vec3 Forward;
            public FixMath.F64Vec3 Side;
            public FixMath.F64Vec3 Up;
            public FixMath.F64 Width;
            public FixMath.F64 Height;
        }
        public FCycleDataBuffer<RectangeObstacleInfo> HitRectangeObstacleBuff = new FCycleDataBuffer<RectangeObstacleInfo>(1024);

        // 缓存历史entity的信息
        public struct EntityDebugInfo
        {
            public FixMath.F64 maxSpeed;
            public FixMath.F64 radius;
            public FixMath.F64Vec3 position;
            public FixMath.F64Vec3 velocity;
            public FixMath.F64Vec3 steerPosition;

            public FixMath.F64Vec3 forward;
            public FixMath.F64Vec3 side;
            public FixMath.F64Vec3 up;
        };
        public FCycleDataBuffer<EntityDebugInfo> EntityInfoBuff = new FCycleDataBuffer<EntityDebugInfo>(1024);

        // Steer force
        public struct SteerForceInfo
        {
            public FixMath.F64Vec3 position;
            public FixMath.F64Vec3 steerForce;
            public FixMath.F64 maxForce;
            
            public FixMath.F64Vec3 obstacleForce;
            public FixMath.F64Vec3 targetForce;
            public FixMath.F64Vec3 avoidNeighborFoce;

            public void reset()
            {
                position = FixMath.F64Vec3.Zero;
                steerForce = FixMath.F64Vec3.Zero;
                obstacleForce = FixMath.F64Vec3.Zero;
                targetForce = FixMath.F64Vec3.Zero;
                avoidNeighborFoce = FixMath.F64Vec3.Zero;
            }
        };
        public FCycleDataBuffer<SteerForceInfo> SteerFoceInfoBuff = new FCycleDataBuffer<SteerForceInfo>(1024);

        // Steer avoid neighbors
        public struct SteerAvoidNeighborInfo
        {
            public FixMath.F64Vec3 threatPosition;
            public FixMath.F64 threatRadius;
            public FixMath.F64Vec3 ourFuturePosition;     // 相交位置(self)
            public FixMath.F64Vec3 threatFuturePosition;  // 相交位置(neighbor)
            public PathIntersection pathIntersection;
        };
        public FCycleDataBuffer<SteerAvoidNeighborInfo> SteerAvoidNeighborInfoBuff = new FCycleDataBuffer<SteerAvoidNeighborInfo>(1024);

        public struct SteerAvoidCloseNeighborInfo
        {
            public FixMath.F64Vec3 threatPosition;
            public FixMath.F64 threatRadius;
            public FixMath.F64Vec3 avoidDirection;
            public IVehicle.FAvoidNeighborInfo avoidNeighborInfo;
        }
        public FCycleDataBuffer<SteerAvoidCloseNeighborInfo> SteerAvoidCloseNeighborInfoBuff = new FCycleDataBuffer<SteerAvoidCloseNeighborInfo>(1024);

        public void Draw(IAnnotationService annotation, MovableEntity vechile, int frameNo, int specailFrame = -1)
        {
            System.Func<int, bool> ShouldDebugerDraw = (InFrameNo) => {
                return specailFrame < 0 ? (frameNo - InFrameNo) <= SteerDebugerDrawFrames : InFrameNo == specailFrame;
            };

            // 绘制相交信息
            PathInterSectionBuff.ForEach((frame, intersection) => {
                if (ShouldDebugerDraw(frame) && intersection.intersect)
                {
                    var point = intersection.surfacePoint;
                    var normal = intersection.surfaceNormal;
                    var steerHint = intersection.steerHint;

                    var dirVecLength = FixMath.F64.FromFloat(0.1f);
                    // draw hit point
                    var pointSize = FixMath.F64Vec3.FromFloat(0.01f, 0.01f, 0.01f);
                    annotation.SolidCube(point, pointSize, Colors.Red, FixMath.F64.One);
                    // draw hit normal
                    var normalEndPoint = point + normal * dirVecLength;
                    annotation.Line(point, normalEndPoint, Colors.Red, FixMath.F64.One);
                    // draw hint direction
                    annotation.Line(normalEndPoint, normalEndPoint + steerHint * dirVecLength, Colors.Green, FixMath.F64.One);
                }
            });

            // 绘制阻挡信息
            var LightCoral = FixMath.F64Vec3.FromFloat(240f / 255f, 128f / 255f, 128f / 255f);
            HitRectangeObstacleBuff.ForEach((frame, hitObstacle) => {
                if (ShouldDebugerDraw(frame))
                {
                    // 绘制平面
                    var planePoint = hitObstacle.Position - hitObstacle.Forward * FixMath.F64.FromFloat(0.1f);
                    annotation.SolidPlane(planePoint, hitObstacle.Forward, new FixMath.F64Vec2(hitObstacle.Width, hitObstacle.Height), LightCoral, FixMath.F64.FromFloat(0.3f));

                    // 绘制法线
                    annotation.Line(hitObstacle.Position, hitObstacle.Position + hitObstacle.Forward * FixMath.F64.Half, Colors.Green, FixMath.F64.One);
                }
            });

            // Entity基础信息
            EntityInfoBuff.ForEach((frame, entity) => {
                if (ShouldDebugerDraw(frame))
                {
                    var movable = new MovableEntity(null, null, null, annotation);
                    movable.Position = entity.position;
                    movable.Forward = entity.forward;
                    movable.Side = entity.side;
                    movable.Up = entity.up;
                    movable.Radius = entity.radius;

                    // draw vehicle
                    Util.Draw.drawBasic2dCircularVehicle(annotation, movable, Colors.Gray50);
                    // draw transform
                    Util.Draw.drawAxes(annotation, movable, FixMath.F64Vec3.FromFloat(1f, 1f, 1f));

                    // draw velocity
                    var r = entity.radius;
                    var u = r * FixMath.F64.FromFloat(0.05f) * FixMath.F64Vec3.Up; // slightly up
                    var p = entity.position;
                    var v = entity.velocity;
                    var speed = v.Length();
                    var maxSpeed = entity.maxSpeed;
                    if (speed > 0)
                    {
                        var offset = FixMath.F64.FromFloat(0.1f) * FixMath.F64Vec3.Up;
                        var len = Utilities.GetMappedRangeValueClamped(new FixMath.F64Vec2(FixMath.F64.Zero, maxSpeed), new FixMath.F64Vec2(FixMath.F64.Zero, r * 2), speed);
                        var vn = v.Normalize();
                        annotation.Line(p + u + offset, p + u + offset + vn * len, Colors.White, FixMath.F64.One);
                    }

                    // draw steer point
                    var steerPosition = entity.steerPosition;
                    var sp = steerPosition + u;
                    var pointSize = FixMath.F64Vec3.FromFloat(0.02f, 0.02f, 0.02f);
                    annotation.SolidCube(sp, pointSize, Colors.Red, FixMath.F64.One);
                    annotation.Line(p + u, steerPosition + u, Colors.Red, FixMath.F64.One);
                }
            });

            // 绘制避让最近Neighbor信息
            SteerAvoidCloseNeighborInfoBuff.ForEach((frame, info) => {
                if (ShouldDebugerDraw(frame))
                {
                    var entity = EntityInfoBuff.FindData(frame);
                    // draw threat entity
                    Util.Draw.drawCircleOrDisk(annotation, info.threatRadius, FixMath.F64Vec3.Up, info.threatPosition, Colors.Yellow, 10, false, false);
                    
                    if (null != entity)
                    {
                        // draw avoid direction
                        var start = entity.Value.position;
                        var end = entity.Value.position + info.avoidDirection * FixMath.F64.FromFloat(1.0f);
                        Util.Draw.drawLine(annotation, start, end, Colors.Yellow);

                        // draw VO
                        var relativePosition = info.threatPosition - entity.Value.position;
                        var distSq = relativePosition.LengthSquared2D();
                        var combinedRadius = info.threatRadius + entity.Value.radius;
                        var combinedRadiusSq = combinedRadius * combinedRadius;

                        FixMath.F64Vec2 u = FixMath.F64Vec2.Zero;
                        FixMath.F64Vec2 left, right;

                        var leg = FixMath.F64.Sqrt(distSq - combinedRadiusSq);

                        left = new FixMath.F64Vec2(relativePosition.X * leg - relativePosition.Y * combinedRadius,
                                relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                        right = -new FixMath.F64Vec2(relativePosition.X * leg + relativePosition.Y * combinedRadius,
                                -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                        // left
                        start = entity.Value.position;
                        end = entity.Value.position + left.Cast(FixMath.F64.Zero) * FixMath.F64.FromFloat(5.0f);
                        Util.Draw.drawLine(annotation, start, end, Colors.Red);
                        // right
                        start = entity.Value.position;
                        end = entity.Value.position + right.Cast(FixMath.F64.Zero) * FixMath.F64.FromFloat(5.0f);
                        Util.Draw.drawLine(annotation, start, end, Colors.Green);
                    }
                }
            });
        }

        public int GetPrevValidDataFrame(int frameNo, eDebugerDataType dataType)
        {
            switch (dataType)
            {
                case eDebugerDataType.PathIntersection:
                    return PathInterSectionBuff.FindNearestPrevFrame(frameNo);
                case eDebugerDataType.LocalBoundary:
                    // return LocalBoundaryBuff.FindNearestPrevFrame(frameNo);
                case eDebugerDataType.EntityInfo:
                    return EntityInfoBuff.FindNearestPrevFrame(frameNo);
                case eDebugerDataType.SteerForceInfo:
                    return SteerFoceInfoBuff.FindNearestPrevFrame(frameNo);
                case eDebugerDataType.SteerAvoidNeighborInfo:
                    return SteerAvoidNeighborInfoBuff.FindNearestPrevFrame(frameNo);
                default:
                    break;
            }
            return frameNo;
        }

        public int GetNextValidDataFrame(int frameNo, eDebugerDataType dataType)
        {
            switch (dataType)
            {
                case eDebugerDataType.PathIntersection:
                    return PathInterSectionBuff.FindNearestNextFrame(frameNo);
                case eDebugerDataType.LocalBoundary:
                    // return LocalBoundaryBuff.FindNearestNextFrame(frameNo);
                case eDebugerDataType.EntityInfo:
                    return EntityInfoBuff.FindNearestNextFrame(frameNo);
                case eDebugerDataType.SteerForceInfo:
                    return SteerFoceInfoBuff.FindNearestNextFrame(frameNo);
                case eDebugerDataType.SteerAvoidNeighborInfo:
                    return SteerAvoidNeighborInfoBuff.FindNearestNextFrame(frameNo);
                default:
                    break;
            }
            return frameNo;
        }
    }
}
