
using Pathfinding.Util;

namespace SharpSteer2.Obstacles
{
    // RectangleObstacle: a rectangular obstacle of a given height, width,
    // position and orientation.  It is a rectangle centered on the XY (aka
    // side/up) plane of a local space.
    public class RectangleObstacle : PlaneObstacle
    {
        public FixMath.F64 width = FixMath.F64.One;  // width  of rectangle centered on local X (side) axis
        public FixMath.F64 height = FixMath.F64.One; // height of rectangle centered on local Y (up)   axis

        // constructors
        public RectangleObstacle(FixMath.F64 w, FixMath.F64 h)
        {
            width = w;
            height = h;
        }

        public RectangleObstacle(FixMath.F64 w, FixMath.F64 h, 
                          FixMath.F64Vec3 s, FixMath.F64Vec3 u, FixMath.F64Vec3 f, FixMath.F64Vec3 p,
                          seenFromState sf) 
        {

            Side = s;
            Up = u;
            Forward = f;
            Position = p;

            width = w;
            height = h;

            setSeenFrom(sf);
        }

        public override void draw(IAnnotationService annotation, bool filled, FixMath.F64Vec3 color, FixMath.F64Vec3 viewpoint)
        {
            if (null != annotation)
            {
                // 绘制平面
                var planePoint = Position - Forward * FixMath.F64.FromFloat(0.1f);
                annotation.SolidPlane(planePoint, Forward, new FixMath.F64Vec2(width, height), color, FixMath.F64.FromFloat(0.3f));

                // 绘制法线
                // annotation.Line(Position, Position + Forward * FixMath.F64.Half, Colors.Green, FixMath.F64.One);

                // 绘制坐标系
                Draw.drawAxes(annotation, this, FixMath.F64Vec3.One);
            }
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Rectangle;
        }

        // determines if a given point on XY plane is inside obstacle shape
        public override bool xyPointInsideShape(FixMath.F64Vec3 point, FixMath.F64 radius)
        {
            var w = radius + (width * FixMath.F64.Half);
            var h = radius + (height * FixMath.F64.Half);
            return !((point.X >  w) || (point.X < -w) || (point.Y >  h) || (point.Y < -h));
        }
    }
}
