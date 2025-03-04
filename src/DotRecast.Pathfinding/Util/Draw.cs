
using SharpSteer2;
using SharpSteer2.Helpers;

namespace DotRecast.Pathfinding.Util
{
    public static class Draw
    {
        public static void drawLine(
            IAnnotationService annotation, 
            FixMath.F64Vec3 startPoint, 
            FixMath.F64Vec3 endPoint, 
            FixMath.F64Vec3 color)
        {
            if (annotation != null)
                annotation.Line(startPoint, endPoint, color, FixMath.F64.One);
        }

        public static void drawLineLifeTime(
            IAnnotationService annotation,
            FixMath.F64Vec3 startPoint,
            FixMath.F64Vec3 endPoint,
            FixMath.F64Vec3 color,
            FixMath.F64 lifeTime)
        {
            if (annotation != null)
                annotation.Line(startPoint, endPoint, color, FixMath.F64.One);
        }


        // ----------------------------------------------------------------------------
        // draw a line with alpha blending

        public static void drawLineAlpha(
            IAnnotationService annotation,
            FixMath.F64Vec3 startPoint,
            FixMath.F64Vec3 endPoint,
            FixMath.F64Vec3 color,
            FixMath.F64 alpha)
        {
            if (annotation != null)
                annotation.Line(startPoint, endPoint, color, alpha);
        }

        // ------------------------------------------------------------------------
        // draw the three axes of a LocalSpace: three lines parallel to the
        // basis vectors of the space, centered at its origin, of lengths
        // given by the coordinates of "size".


        public static void drawAxes(
            IAnnotationService annotation, 
            ILocalSpaceBasis localSpace,
            FixMath.F64Vec3 size)
        {
            var x = new FixMath.F64Vec3(size.X / 2, FixMath.F64.Zero, FixMath.F64.Zero);
            var y = new FixMath.F64Vec3(FixMath.F64.Zero, size.Y / 2, FixMath.F64.Zero);
            var z = new FixMath.F64Vec3(FixMath.F64.Zero, FixMath.F64.Zero, size.Z / 2);

            drawLine(annotation, localSpace.Position, localSpace.GlobalizePosition(x), Colors.Red);
            drawLine(annotation, localSpace.Position, localSpace.GlobalizePosition(y), Colors.Green);
            drawLine(annotation, localSpace.Position, localSpace.GlobalizePosition(z), Colors.Blue);
        }

        public static void drawCircleOrDisk(
            IAnnotationService annotation,
            FixMath.F64 radius,
            FixMath.F64Vec3 axis,
            FixMath.F64Vec3 center,
            FixMath.F64Vec3 color,
            int segments,
            bool filled,
            bool in3d)
        {
            annotation.CircleOrDisk3D(radius, center, axis, color, segments, filled);
        }

        public static void drawBasic2dCircularVehicle(
            IAnnotationService annotation,
            IVehicle vehicle,
            FixMath.F64Vec3 color)
        {
            // "aspect ratio" of body (as seen from above)
            var x = FixMath.F64.Half;
            var y = Utilities.sqrtXXX(1 - (x * x));

            // radius and position of vehicle
            var r = vehicle.Radius;
            var p = vehicle.Position;

            // shape of triangular body
            var u = r * FixMath.F64.FromFloat(0.05f) * FixMath.F64Vec3.Up; // slightly up
            var f = r * vehicle.Forward;
            var s = r * vehicle.Side * x;
            var b = r * vehicle.Forward * -y;

            var verts = new FixMath.F64Vec3[] {
                p + f + u,
                p + b - s + u,
                p + b + s + u,
            };

            drawLine(annotation, verts[0], verts[1], color);
            drawLine(annotation, verts[1], verts[2], color);
            drawLine(annotation, verts[2], verts[0], color);

            // draw the circular collision boundary
            drawCircleOrDisk(annotation, r, FixMath.F64Vec3.Up, p + u, Colors.White, 20, false, false);
        }
    }
}
