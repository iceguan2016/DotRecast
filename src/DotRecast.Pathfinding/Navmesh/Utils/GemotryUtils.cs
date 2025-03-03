using System;
using System.Collections.Generic;
using System.Text;
using DotRecast.Core.Numerics;


namespace Navmesh.Utils
{
    public static class GemotryUtils
    {

        public static UnityEngine.Vector3 ToUnityVec3(this hxDaedalus.data.math.Point2D InPoint2D, double InHeight=0.0)
        {
            return new UnityEngine.Vector3((float)InPoint2D.x, (float)InHeight, (float)InPoint2D.y);
        }

        public static RcVec3f ToRecastVec3(this hxDaedalus.data.math.Point2D InPoint2D, double InHeight = 0.0)
        {
            return new RcVec3f((float)InPoint2D.x, (float)InHeight, (float)InPoint2D.y);
        }
    }
}
