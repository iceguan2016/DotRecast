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
        public float width = 1.0f;  // width  of box centered on local X (side)    axis
        public float height = 1.0f; // height of box centered on local Y (up)      axis
        public float depth = 1.0f;  // depth  of box centered on local Z (forward) axis

        // constructors
        BoxObstacle(float w, float h, float d)
        {
            width = w;
            height = h;
            depth = d;
        }

        public override void draw(bool filled, Vector4 color, Vector3 viewpoint)
        {
        }

        public override void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi)
        {
            // abbreviations
            float w = width; // dimensions
            float h = height;
            float d = depth;
            Vector3 s = Side; // local space
            Vector3 u = Up;
            Vector3 f = Forward;
            Vector3 p = Position;
            Vector3 hw = s * (0.5f * width); // offsets for face centers
            Vector3 hh = u * (0.5f * height);
            Vector3 hd = f * (0.5f * depth);
            seenFromState sf = seenFrom();

            // the box's six rectangular faces
            RectangleObstacle r1 = new RectangleObstacle(w, h, s, u, f, p + hd, sf); // front
            RectangleObstacle r2 = new RectangleObstacle(w, h, -s,  u, -f, p - hd, sf); // back
            RectangleObstacle r3 = new RectangleObstacle(d, h, -f,  u,  s, p + hw, sf); // side
            RectangleObstacle r4 = new RectangleObstacle(d, h, f, u, -s, p - hw, sf); // other side
            RectangleObstacle r5 = new RectangleObstacle(w, d, s, -f,  u, p + hh, sf); // top
            RectangleObstacle r6 = new RectangleObstacle(w, d, -s, -f, -u, p - hh, sf); // bottom

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
                                (pi.vehicleOutside ? 1.0f : -1.0f));
            }
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Box;
        }
    }
}
