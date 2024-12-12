using System;
using System.ComponentModel;
using System.Numerics;
using SharpSteer2.Helpers;

namespace SharpSteer2.Obstacles
{
    // PlaneObstacle: a planar obstacle of a given position and orientation.
    // The plane is defined as the XY (aka side/up) plane of a local space.
    // The +Z (forward) half-space is considered "outside" the obstacle.  
    //
    // This is also the base class for several other obstacles which represent
    // 2d shapes (rectangle, triangle, ...) arbitarily oriented and positioned
    // in 2d space.  They specialize this class via xyPointInsideShape which
    // tests if a given point on the XZ plane is inside the obstacle's shape.
    public class PlaneObstacle : LocalSpaceObstacle
    {
        public override void draw(bool filled, Vector4 color, Vector3 viewpoint)
        {
        }

        public override void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi)
        {
            // initialize pathIntersection object to "no intersection found"
            pi.intersect = false;

            Vector3 lp = this.LocalizePosition(vehicle.Position);
            Vector3 ld = this.LocalizeDirection(vehicle.Forward);

            // no obstacle intersection if path is parallel to XY (side/up) plane
            if (ld.Z == 0.0f) return;
            if (ld.Dot(Vector3Helpers.Forward) == 0.0f) return;

            // no obstacle intersection if vehicle is heading away from the XY plane
            if ((lp.Z > 0.0f) && (ld.Z > 0.0f)) return;
            if ((lp.Z < 0.0f) && (ld.Z < 0.0f)) return;

            // no obstacle intersection if obstacle "not seen" from vehicle's side
            if ((seenFrom() == seenFromState.outside) && (lp.Z < 0.0f)) return;
            if ((seenFrom() == seenFromState.inside) && (lp.Z > 0.0f)) return;

            // find intersection of path with rectangle's plane (XY plane)
            float ix = lp.X - (ld.X * lp.Z / ld.Z);
            float iy = lp.Y - (ld.Y * lp.Z / ld.Z);
            Vector3 planeIntersection = new Vector3(ix, iy, 0.0f);

            // no obstacle intersection if plane intersection is outside 2d shape
            if (!xyPointInsideShape(planeIntersection, vehicle.Radius)) return;

            // otherwise, the vehicle path DOES intersect this rectangle
            Vector3 localXYradial = planeIntersection.Normalize();
            Vector3 radial = this.GlobalizeDirection(localXYradial);
            float sideSign = (lp.Z >= -0.001f) ? +1.0f : -1.0f;
            Vector3 opposingNormal = this.Forward * sideSign;
            pi.intersect = true;
            pi.obstacle = this;
            pi.distance = (lp - planeIntersection).Length();
            pi.steerHint = opposingNormal + radial; // should have "toward edge" term?
            pi.surfacePoint = this.GlobalizePosition(planeIntersection);
            pi.surfaceNormal = opposingNormal;
            pi.vehicleOutside = lp.Z > 0.0f;
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Plane;
        }

        // determines if a given point on XY plane is inside obstacle shape
        public virtual bool xyPointInsideShape(Vector3 point, float radius)
        {
            return true; // always true for PlaneObstacle
        }
    }
}
