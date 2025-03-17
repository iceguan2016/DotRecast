
using System;
using System.Collections.Generic;

namespace Pathfinding.Triangulation.Data
{
    public class Vertex
    {
        static Vertex()
        {
            Vertex.INC = 0;
        }

        public Vertex(global::haxe.lang.EmptyObject empty)
        {
        }


        public Vertex()
        {
            colorDebug = -1;
            _id = INC;
            INC++;

            _pos = FixMath.F64Vec2.Zero;
            _fromConstraintSegments = new List<ConstraintSegment>();
        }

        public static int INC;

        public int _id;

        public FixMath.F64Vec2 _pos;

        public bool _isReal;

        public Edge _edge;

        public List<ConstraintSegment> _fromConstraintSegments;

        public int colorDebug;

        public int get_id()
        {
            return this._id;
        }


        public bool get_isReal()
        {
            return this._isReal;
        }


        public FixMath.F64Vec2 get_pos()
        {
            return this._pos;
        }


        public List<ConstraintSegment> get_fromConstraintSegments()
        {
            return this._fromConstraintSegments;
        }

        public List<ConstraintSegment> set_fromConstraintSegments(List<ConstraintSegment> value)
        {
            return this._fromConstraintSegments = value;
        }


        public void setDatas(Edge edge, bool isReal = true)
        {
            this._isReal = isReal;
            this._edge = edge;
        }


        public void addFromConstraintSegment(ConstraintSegment segment)
        {
            if (!this._fromConstraintSegments.Contains(segment))
            {
                this._fromConstraintSegments.Add(segment);
            }
        }


        public void removeFromConstraintSegment(ConstraintSegment segment)
        {
            this._fromConstraintSegments.Remove(segment);
        }


        public void dispose()
        {
            this._pos = FixMath.F64Vec2.Zero;
            this._edge = null;
            this._fromConstraintSegments.Clear();
        }


        public Edge get_edge()
        {
            return this._edge;
        }


        public Edge set_edge(Edge value)
        {
            return this._edge = value;
        }


        public string toString()
        {
            return "ver_id " + get_id();
        }
    }
}
