// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System;
using System.Numerics;
using SharpSteer2.Helpers;

namespace SharpSteer2.Obstacles
{
	/// <summary>
	/// SphericalObstacle a simple concrete type of obstacle.
	/// </summary>
	public class SphericalObstacle : Obstacle
    {
	    public float Radius;
	    public Vector3 Center;

	    public SphericalObstacle()
			: this(1, Vector3.Zero)
		{
		}

        public SphericalObstacle(float r, Vector3 c)
		{
			Radius = r;
			Center = c;
		}

        public override void findIntersectionWithVehiclePath(BaseVehicle vehicle, ref PathIntersection pi)
        {
            // This routine is based on the Paul Bourke's derivation in:
            //   Intersection of a Line and a Sphere (or circle)
            //   http://www.swin.edu.au/astronomy/pbourke/geometry/sphereline/
            // But the computation is done in the vehicle's local space, so
            // the line in question is the Z (Forward) axis of the space which
            // simplifies some of the calculations.

            float b, c, d, p, q, s;
            Vector3 lc = Vector3.Zero;

            // initialize pathIntersection object to "no intersection found"
            pi.intersect = false;

            // find sphere's "local center" (lc) in the vehicle's coordinate space
            lc = vehicle.LocalizePosition(Center);
            pi.vehicleOutside = lc.Length() > Radius;

            // if obstacle is seen from inside, but vehicle is outside, must avoid
            // (noticed once a vehicle got outside it ignored the obstacle 2008-5-20)
            if (pi.vehicleOutside && (seenFrom() == seenFromState.inside))
            {
                pi.intersect = true;
                pi.distance = 0.0f;
                pi.steerHint = (Center - vehicle.Position).Normalize();
                return;
            }

            // compute line-sphere intersection parameters
            float r = Radius + vehicle.Radius;
            b = -2 * lc.Z;
            c = Utilities.square(lc.X) + Utilities.square(lc.Y) + Utilities.square(lc.Z) - Utilities.square(r);
            d = (b * b) - (4 * c);

            // when the path does not intersect the sphere
            if (d < 0) return;

            // otherwise, the path intersects the sphere in two points with
            // parametric coordinates of "p" and "q".  (If "d" is zero the two
            // points are coincident, the path is tangent)
            s = Utilities.sqrtXXX(d);
            p = (-b + s) / 2;
            q = (-b - s) / 2;

            // both intersections are behind us, so no potential collisions
            if ((p < 0) && (q < 0)) return;

            // at least one intersection is in front, so intersects our forward
            // path
            pi.intersect = true;
            pi.obstacle = this;
            pi.distance =
                ((p > 0) && (q > 0)) ?
                // both intersections are in front of us, find nearest one
                ((p < q) ? p : q) :
                // otherwise one is ahead and one is behind: we are INSIDE obstacle
                (seenFrom() ==  seenFromState.outside?
                 // inside a solid obstacle, so distance to obstacle is zero
                 0.0f :
                 // hollow obstacle (or "both"), pick point that is in front
                 ((p > 0) ? p : q));
            pi.surfacePoint =
                vehicle.Position + (vehicle.Forward * pi.distance);
            pi.surfaceNormal = (pi.surfacePoint - Center).Normalize();
            switch (seenFrom())
            {
                case seenFromState.outside:
                    pi.steerHint = pi.surfaceNormal;
                    break;
                case seenFromState.inside:
                    pi.steerHint = -pi.surfaceNormal;
                    break;
                case seenFromState.both:
                    pi.steerHint = pi.surfaceNormal * (pi.vehicleOutside ? 1.0f : -1.0f);
                    break;
            }
        }

        public override void draw(bool filled, Vector4 color, Vector3 viewpoint)
        {
        }

        public override ObstacleType getObstacleType()
        {
            return ObstacleType.Sphere;
        }
    }
}
