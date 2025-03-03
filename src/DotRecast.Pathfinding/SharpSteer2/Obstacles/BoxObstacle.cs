using System;
using System.ComponentModel;
using System.Numerics;
using SharpSteer2.Helpers;

namespace SharpSteer2.Obstacles
{
    // BoxObstacle: a box-shaped (cuboid) obstacle of a given height, width,
    // depth, position and orientation.  The box is centered on and aligned
    // with a local space.
    public class BoxObstacle : LocalSpaceObstacle
    {
        public FixMath.F64 width = FixMath.F64.One;  // width  of box centered on local X (side)    axis
        public FixMath.F64 height = FixMath.F64.One; // height of box centered on local Y (up)      axis
        public FixMath.F64 depth = FixMath.F64.One;  // depth  of box centered on local Z (forward) axis

        // constructors
        BoxObstacle(FixMath.F64 w, FixMath.F64 h, FixMath.F64 d)
        {
            width = w;
            height = h;
            depth = d;
        }

        public override void draw(bool filled, FixMath.F64Vec3 color, FixMath.F64Vec3 viewpoint)
        {
        }

        public override void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi)
        {
            // abbreviations
            var w = width; // dimensions
            var h = height;
            var d = depth;
            var s = Side; // local space
            var u = Up;
            var f = Forward;
            var p = Position;
            var hw = s * (FixMath.F64.Half * width); // offsets for face centers
            var hh = u * (FixMath.F64.Half * height);
            var hd = f * (FixMath.F64.Half * depth);
            seenFromState sf = seenFrom();

            // the box's six rectangular faces
            var r1 = new RectangleObstacle(w, h, s, u, f, p + hd, sf); // front
            var r2 = new RectangleObstacle(w, h, -s,  u, -f, p - hd, sf); // back
            var r3 = new RectangleObstacle(d, h, -f,  u,  s, p + hw, sf); // side
            var r4 = new RectangleObstacle(d, h, f, u, -s, p - hw, sf); // other side
            var r5 = new RectangleObstacle(w, d, s, -f,  u, p + hh, sf); // top
            var r6 = new RectangleObstacle(w, d, -s, -f, -u, p - hh, sf); // bottom

            // group the six RectangleObstacle faces together
            ObstacleGroup faces = new ObstacleGroup();
            faces.Add(r1);
            faces.Add(r2);
            faces.Add(r3);
            faces.Add(r4);
            faces.Add(r5);
            faces.Add(r6);

            // find first intersection of vehicle path with group of six faces
            Obstacle.firstPathIntersectionWithObstacleGroup(vehicle, faces, out pi, out var next);

            // when intersection found, adjust PathIntersection for the box case
            if (pi.intersect)
            {
                pi.obstacle = this;
                pi.steerHint = ((pi.surfacePoint - Position).Normalize() *
                                (pi.vehicleOutside ? FixMath.F64.One : -FixMath.F64.One));
            }
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Box;
        }
    }
}
