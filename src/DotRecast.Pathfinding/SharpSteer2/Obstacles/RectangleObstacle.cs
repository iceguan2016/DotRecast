using System;
using System.ComponentModel;
using System.Numerics;
using SharpSteer2.Helpers;

namespace SharpSteer2.Obstacles
{
    // RectangleObstacle: a rectangular obstacle of a given height, width,
    // position and orientation.  It is a rectangle centered on the XY (aka
    // side/up) plane of a local space.
    public class RectangleObstacle : PlaneObstacle
    {
        float width = 1.0f;  // width  of rectangle centered on local X (side) axis
        float height = 1.0f; // height of rectangle centered on local Y (up)   axis

        // constructors
        public RectangleObstacle(float w, float h)
        {
            width = w;
            height = h;
        }

        public RectangleObstacle(float w, float h, 
                          Vector3 s, Vector3 u, Vector3 f, Vector3 p,
                          seenFromState sf) 
        {

            Side = s;
            Up = u;
            Forward = f;
            Position = p;

            setSeenFrom(sf);
        }

        public override void draw(bool filled, Vector4 color, Vector3 viewpoint)
        {
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Rectangle;
        }

        // determines if a given point on XY plane is inside obstacle shape
        public override bool xyPointInsideShape(Vector3 point, float radius)
        {
            float w = radius + (width * 0.5f);
            float h = radius + (height * 0.5f);
            return !((point.X >  w) || (point.X < -w) || (point.Y >  h) || (point.Y < -h));
        }
    }
}
