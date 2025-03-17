using System;
using System.Collections.Generic;

namespace Pathfinding.Triangulation.Data
{
    public class ConstraintShape
    {
        static ConstraintShape()
        {
            ConstraintShape.INC = 0;
        }

        public ConstraintShape()
        {
            _id = INC;
            INC++;
            segments = new List<ConstraintSegment>();
        }

        public static int INC;

        public List<ConstraintSegment> segments;
        public int _id;

        public virtual int get_id()
        {
            return this._id;
        }


        public virtual void dispose()
        {
            while ((segments.Count > 0))
            {
                var index = segments.Count - 1;
                segments[index].dispose();
                segments.RemoveAt(index);
            }
            this.segments.Clear();
        }
    }
}
