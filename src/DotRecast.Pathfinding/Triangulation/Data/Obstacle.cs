using System;
using System.Collections.Generic;
using Pathfinding.Triangulation.Math;

namespace Pathfinding.Triangulation.Data
{
    public class Obstacle
    {
        static Obstacle()
        {
            Obstacle.INC = 0;
        }

        public Obstacle()
        {
            _id = INC;
            INC++;

            _pivotX = FixMath.F64.Zero;
            _pivotY = FixMath.F64.Zero;

            _matrix = new Matrix2D();
            _scaleX = FixMath.F64.One;
            _scaleY = FixMath.F64.One;
            _rotation = FixMath.F64.Zero;
            _x = FixMath.F64.Zero;
            _y = FixMath.F64.Zero;

            _coordinates = new List<FixMath.F64>();

            _hasChanged = false;
        }

        public static int INC;

        public int _id;

        public Matrix2D _matrix;

        public List<FixMath.F64> _coordinates;

        public ConstraintShape _constraintShape;

        public FixMath.F64 _pivotX;

        public FixMath.F64 _pivotY;

        public FixMath.F64 _scaleX;

        public FixMath.F64 _scaleY;

        public FixMath.F64 _rotation;

        public FixMath.F64 _x;

        public FixMath.F64 _y;

        public bool _hasChanged;

        public int get_id()
        {
            return this._id;
        }


        public void dispose()
        {
            this._coordinates = null;
            this._constraintShape = null;
        }


        public void updateValuesFromMatrix()
        {
        }


        public void updateMatrixFromValues()
        {
            this._matrix.identity();
            this._matrix.translate(-(this._pivotX), -(this._pivotY));
            this._matrix.scale(this._scaleX, this._scaleY);
            this._matrix.rotate(this._rotation);
            this._matrix.translate(this._x, this._y);
        }


        public FixMath.F64 get_pivotX()
        {
            return this._pivotX;
        }


        public FixMath.F64 set_pivotX(FixMath.F64 @value)
        {
            this._pivotX = @value;
            this._hasChanged = true;
            return @value;
        }


        public FixMath.F64 get_pivotY()
        {
            return this._pivotY;
        }


        public FixMath.F64 set_pivotY(FixMath.F64 @value)
        {
            this._pivotY = @value;
            this._hasChanged = true;
            return @value;
        }


        public FixMath.F64 get_scaleX()
        {
            return this._scaleX;
        }


        public FixMath.F64 set_scaleX(FixMath.F64 @value)
        {
            if ((this._scaleX != @value))
            {
                this._scaleX = @value;
                this._hasChanged = true;
            }

            return @value;
        }


        public FixMath.F64 get_scaleY()
        {
            return this._scaleY;
        }


        public FixMath.F64 set_scaleY(FixMath.F64 @value)
        {
            if ((this._scaleY != @value))
            {
                this._scaleY = @value;
                this._hasChanged = true;
            }

            return @value;
        }


        public FixMath.F64 get_rotation()
        {
            return this._rotation;
        }


        public FixMath.F64 set_rotation(FixMath.F64 @value)
        {
            if ((this._rotation != @value))
            {
                this._rotation = @value;
                this._hasChanged = true;
            }

            return @value;
        }


        public FixMath.F64 get_x()
        {
            return this._x;
        }


        public FixMath.F64 set_x(FixMath.F64 @value)
        {
            if ((this._x != @value))
            {
                this._x = @value;
                this._hasChanged = true;
            }

            return @value;
        }


        public FixMath.F64 get_y()
        {
            return this._y;
        }


        public FixMath.F64 set_y(FixMath.F64 @value)
        {
            if ((this._y != @value))
            {
                this._y = @value;
                this._hasChanged = true;
            }

            return @value;
        }


        public Matrix2D get_matrix()
        {
            return this._matrix;
        }


        public Matrix2D set_matrix(Matrix2D @value)
        {
            this._matrix = @value;
            this._hasChanged = true;
            return @value;
        }


        public List<FixMath.F64> get_coordinates()
        {
            return this._coordinates;
        }


        public List<FixMath.F64> set_coordinates(List<FixMath.F64> @value)
        {
            this._coordinates = @value;
            this._hasChanged = true;
            return @value;
        }


        public ConstraintShape get_constraintShape()
        {
            return this._constraintShape;
        }


        public ConstraintShape set_constraintShape(ConstraintShape @value)
        {
            this._constraintShape = @value;
            this._hasChanged = true;
            return @value;
        }


        public bool get_hasChanged()
        {
            return this._hasChanged;
        }


        public bool set_hasChanged(bool @value)
        {
            this._hasChanged = @value;
            return @value;
        }


        public List<Edge> get_edges()
        {
            var res = new List<Edge>();
            var seg = _constraintShape.segments;
            for (var i = 0; i < seg.Count; ++i)
            {
                for (var j = 0; j < seg[i]._edges.Count; ++j)
                {
                    res.Add(seg[i]._edges[j]);
                }
            }

            return res;
        }

        public bool contain_point(FixMath.F64Vec2 p)
        {
            var matrix = new Matrix2D();
            matrix.identity();
            matrix.translate(-this._x, -this._y);
            matrix.rotate(-this._rotation);

            var local_p = matrix.tranform(p);

            if (FixMath.F64.Abs(local_p.X) <= (this._scaleX + Constants.EPSILON) && 
                FixMath.F64.Abs(local_p.Y) <= (this._scaleY + Constants.EPSILON))
            {
                return true;
            }
            return false;
        }
    }
}
