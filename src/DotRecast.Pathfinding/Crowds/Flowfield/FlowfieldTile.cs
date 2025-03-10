using System;
using System.Collections.Generic;
using System.Text;

namespace Pathfinding.Crowds.Flowfield
{
    public class FlowfieldTile
    {
        public FixMath.F64Vec3 BoundMin { get; private set; }
        public FixMath.F64Vec3 BoundMax { get; private set; }



        public FlowfieldTile(FixMath.F64Vec3 InBoundMin, FixMath.F64Vec3 InBoundMax)
        {
            BoundMin = InBoundMin;
            BoundMax = InBoundMax;
        }

        // Face
        public void AddTriangle(FixMath.F64Vec3 v0, FixMath.F64Vec3 v1, FixMath.F64Vec3 v2)
        {
            
        }


    }
}
