using System;

namespace Pathfinding.Triangulation.Math
{
    public struct Matrix2D
    {
        /*    
        represents row vector in homogeneous coordinates:
        [x, y, 1]
    
        Matrix2D represents transform matrix in homogeneous coordinates:
        [a, b, 0]
        [c, d, 0]
        [e, f, 1]
                                  [a, b, 0]
        [x', y', 1] = [x, y, 1] * [c, d, 0]
                                  [e, f, 1]
        */
        public Matrix2D(FixMath.F64? a_ = null, FixMath.F64? b_ = null, FixMath.F64? c_ = null, FixMath.F64? d_ = null, FixMath.F64? e_ = null, FixMath.F64? f_ = null)
        {
            a = a_ ?? FixMath.F64.One;
            b = b_ ?? FixMath.F64.Zero;
            c = c_ ?? FixMath.F64.Zero;
            d = d_ ?? FixMath.F64.One;
            e = e_ ?? FixMath.F64.Zero;
            f = f_ ?? FixMath.F64.Zero;
        }

        public FixMath.F64 a;

        public FixMath.F64 b;

        public FixMath.F64 c;

        public FixMath.F64 d;

        public FixMath.F64 e;

        public FixMath.F64 f;

        public void identity()
        {
            a = FixMath.F64.One;
            b = FixMath.F64.Zero;
            c = FixMath.F64.Zero;
            d = FixMath.F64.One;
            e = FixMath.F64.Zero;
            f = FixMath.F64.Zero;
        }

        public void translate(FixMath.F64 tx, FixMath.F64 ty)
        {
            this.e += tx;
            this.f += ty;
        }


        public void scale(FixMath.F64 sx, FixMath.F64 sy)
        {
            this.a *= sx;
            this.b *= sy;
            this.c *= sx;
            this.d *= sy;
            this.e *= sx;
            this.f *= sy;
        }


        public void rotate(FixMath.F64 rad)
        {
            var cos = FixMath.F64.Cos(rad);
            var sin = FixMath.F64.Sin(rad);
            var a_ = ((this.a * cos) + (this.b * -(sin)));
            var b_ = ((this.a * sin) + (this.b * cos));
            var c_ = ((this.c * cos) + (this.d * -(sin)));
            var d_ = ((this.c * sin) + (this.d * cos));
            var e_ = ((this.e * cos) + (this.f * -(sin)));
            var f_ = ((this.e * sin) + (this.f * cos));
            this.a = a_;
            this.b = b_;
            this.c = c_;
            this.d = d_;
            this.e = e_;
            this.f = f_;
        }

        public FixMath.F64Vec2 tranform(FixMath.F64Vec2 point)
        {
            var x = a * point.X + c * point.Y + e;
            var y = b * point.X + d * point.Y + f;
            return new FixMath.F64Vec2(x, y);
        }


        public FixMath.F64 transformX(FixMath.F64 x, FixMath.F64 y)
        {
            return (((this.a * x) + (this.c * y)) + this.e);
        }


        public FixMath.F64 transformY(FixMath.F64 x, FixMath.F64 y)
        {
            return (((this.b * x) + (this.d * y)) + this.f);
        }


        public void concat(Matrix2D matrix)
        {
            var a_ = ((this.a * matrix.a) + (this.b * matrix.c));
            var b_ = ((this.a * matrix.b) + (this.b * matrix.d));
            var c_ = ((this.c * matrix.a) + (this.d * matrix.c));
            var d_ = ((this.c * matrix.b) + (this.d * matrix.d));
            var e_ = (((this.e * matrix.a) + (this.f * matrix.c)) + matrix.e);
            var f_ = (((this.e * matrix.b) + (this.f * matrix.d)) + matrix.f);
            this.a = a_;
            this.b = b_;
            this.c = c_;
            this.d = d_;
            this.e = e_;
            this.f = f_;
        }
    }
}
