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
        public override void draw(bool filled, FixMath.F64Vec3 color, FixMath.F64Vec3 viewpoint)
        {
        }

        public override void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi)
        {
            // initialize pathIntersection object to "no intersection found"
            pi.intersect = false;

            var lp = this.LocalizePosition(vehicle.Position);
            var ld = this.LocalizeDirection(vehicle.Forward);

            // no obstacle intersection if path is parallel to XY (side/up) plane
            if (ld.Z == FixMath.F64.Zero) return;
            if (ld.Dot(Vector3Helpers.Forward) == FixMath.F64.Zero) return;

            // no obstacle intersection if vehicle is heading away from the XY plane
            if ((lp.Z > FixMath.F64.Zero) && (ld.Z > FixMath.F64.Zero)) return;
            if ((lp.Z < FixMath.F64.Zero) && (ld.Z < FixMath.F64.Zero)) return;

            // no obstacle intersection if obstacle "not seen" from vehicle's side
            if ((seenFrom() == seenFromState.outside) && (lp.Z < FixMath.F64.Zero)) return;
            if ((seenFrom() == seenFromState.inside) && (lp.Z > FixMath.F64.Zero)) return;

            // find intersection of path with rectangle's plane (XY plane)
            var ix = lp.X - (ld.X * lp.Z / ld.Z);
            var iy = lp.Y - (ld.Y * lp.Z / ld.Z);
            var planeIntersection = new FixMath.F64Vec3(ix, iy, FixMath.F64.Zero);

            // no obstacle intersection if plane intersection is outside 2d shape
            if (!xyPointInsideShape(planeIntersection, vehicle.Radius)) return;

            // otherwise, the vehicle path DOES intersect this rectangle
            var localXYradial = planeIntersection.Normalize();
            var radial = this.GlobalizeDirection(localXYradial);
            var sideSign = (lp.Z >= -FixMath.F64.Epsilon) ? FixMath.F64.One : -FixMath.F64.One;
            var opposingNormal = this.Forward * sideSign;
            pi.intersect = true;
            pi.obstacle = this;
            pi.distance = FixMath.F64Vec3.LengthFast(lp - planeIntersection);
            pi.steerHint = opposingNormal + radial; // should have "toward edge" term?
            pi.surfacePoint = this.GlobalizePosition(planeIntersection);
            pi.surfaceNormal = opposingNormal;
            pi.vehicleOutside = lp.Z > FixMath.F64.Zero;
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Plane;
        }

        // determines if a given point on XY plane is inside obstacle shape
        public virtual bool xyPointInsideShape(FixMath.F64Vec3 point, FixMath.F64 radius)
        {
            return true; // always true for PlaneObstacle
        }
    }
}
