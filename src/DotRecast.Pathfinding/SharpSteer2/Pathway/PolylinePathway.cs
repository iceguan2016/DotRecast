// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System.Collections.Generic;
using System.Numerics;

namespace SharpSteer2.Pathway
{
	/// <summary>
	/// PolylinePathway: a simple implementation of the Pathway protocol.  The path
	/// is a "polyline" a series of line segments between specified points.  A
	/// radius defines a volume for the path which is the union of a sphere at each
	/// point and a cylinder along each segment.
	/// </summary>
	public class PolylinePathway : IPathway
	{
	    public int PointCount { get; private set; }

	    private readonly FixMath.F64Vec3[] _points;
	    public IEnumerable<FixMath.F64Vec3> Points
	    {
	        get
	        {
	            return _points;
	        }
	    }

	    public FixMath.F64 Radius { get; private set; }
	    public bool Cyclic { get; private set; }

	    private readonly FixMath.F64[] _lengths;
	    private readonly FixMath.F64Vec3[] _tangents;

	    public FixMath.F64 TotalPathLength { get; private set; }

		/// <summary>
        /// construct a PolylinePathway given the number of points (vertices),
        /// an array of points, and a path radius.
		/// </summary>
		/// <param name="points"></param>
		/// <param name="radius"></param>
		/// <param name="cyclic"></param>
        public PolylinePathway(IList<FixMath.F64Vec3> points, FixMath.F64 radius, bool cyclic)
		{
            // set data members, allocate arrays
            Radius = radius;
            Cyclic = cyclic;
            PointCount = points.Count;
            TotalPathLength = FixMath.F64.Zero;
            if (Cyclic)
                PointCount++;
            _lengths = new FixMath.F64[PointCount];
            _points = new FixMath.F64Vec3[PointCount];
            _tangents = new FixMath.F64Vec3[PointCount];

            // loop over all points
            for (int i = 0; i < PointCount; i++)
            {
                // copy in point locations, closing cycle when appropriate
                bool closeCycle = Cyclic && (i == PointCount - 1);
                int j = closeCycle ? 0 : i;
                _points[i] = points[j];

                // for the end of each segment
                if (i > 0)
                {
                    // compute the segment length
                    _tangents[i] = _points[i] - _points[i - 1];
                    _lengths[i] = FixMath.F64Vec3.LengthFast(_tangents[i]);

                    // find the normalized vector parallel to the segment
                    _tangents[i] *= 1 / _lengths[i];

                    // keep running total of segment lengths
                    TotalPathLength += _lengths[i];
                }
            }
		}

        public FixMath.F64Vec3 MapPointToPath(FixMath.F64Vec3 point, out FixMath.F64Vec3 tangent, out FixMath.F64 outside)
		{
            var minDistance = FixMath.F64.MaxValue;
            var onPath = FixMath.F64Vec3.Zero;
			tangent = FixMath.F64Vec3.Zero;

			// loop over all segments, find the one nearest to the given point
			for (int i = 1; i < PointCount; i++)
			{
			    FixMath.F64Vec3 chosen;
			    FixMath.F64 segmentProjection;
                var d = PointToSegmentDistance(point, _points[i - 1], _points[i], _tangents[i], _lengths[i], out chosen, out segmentProjection);
				if (d < minDistance)
				{
					minDistance = d;
                    onPath = chosen;
                    tangent = _tangents[i];
				}
			}

			// measure how far original point is outside the Pathway's "tube"
			outside = FixMath.F64Vec3.Distance(onPath, point) - Radius;

			// return point on path
			return onPath;
		}

        public FixMath.F64 MapPointToPathDistance(FixMath.F64Vec3 point)
		{
            var minDistance = FixMath.F64.MaxValue;
			var segmentLengthTotal = FixMath.F64.Zero;
			var pathDistance = FixMath.F64.Zero;

			for (int i = 1; i < PointCount; i++)
			{
			    FixMath.F64Vec3 chosen;
			    FixMath.F64 segmentProjection;
                var d = PointToSegmentDistance(point, _points[i - 1], _points[i], _tangents[i], _lengths[i], out chosen, out segmentProjection);
				if (d < minDistance)
				{
					minDistance = d;
                    pathDistance = segmentLengthTotal + segmentProjection;
				}
                segmentLengthTotal += _lengths[i];
			}

			// return distance along path of onPath point
			return pathDistance;
		}

        public FixMath.F64Vec3 MapPathDistanceToPoint(FixMath.F64 pathDistance)
		{
			// clip or wrap given path distance according to cyclic flag
			var remaining = pathDistance;
			if (Cyclic)
			{
				remaining = pathDistance % TotalPathLength;
			}
			else
			{
                if (pathDistance < 0) return _points[0];
                if (pathDistance >= TotalPathLength) return _points[PointCount - 1];
			}

			// step through segments, subtracting off segment lengths until
			// locating the segment that contains the original pathDistance.
			// Interpolate along that segment to find 3d point value to return.
			var result = FixMath.F64Vec3.Zero;
			for (int i = 1; i < PointCount; i++)
			{
                if (_lengths[i] < remaining)
				{
                    remaining -= _lengths[i];
				}
				else
				{
                    var ratio = remaining / _lengths[i];
                    result = FixMath.F64Vec3.Lerp(_points[i - 1], _points[i], ratio);
					break;
				}
			}
			return result;
		}

	    private static FixMath.F64 PointToSegmentDistance(FixMath.F64Vec3 point, FixMath.F64Vec3 ep0, FixMath.F64Vec3 ep1, FixMath.F64Vec3 segmentTangent, FixMath.F64 segmentLength, out FixMath.F64Vec3 chosen, out FixMath.F64 segmentProjection)
		{
			// convert the test point to be "local" to ep0
			var local = point - ep0;

			// find the projection of "local" onto "tangent"
            segmentProjection = FixMath.F64Vec3.Dot(segmentTangent, local);

			// handle boundary cases: when projection is not on segment, the
			// nearest point is one of the endpoints of the segment
			if (segmentProjection < 0)
			{
				chosen = ep0;
				segmentProjection = FixMath.F64.Zero;
				return FixMath.F64Vec3.Distance(point, ep0);
			}
            if (segmentProjection > segmentLength)
			{
				chosen = ep1;
                segmentProjection = segmentLength;
				return FixMath.F64Vec3.Distance(point, ep1);
			}

			// otherwise nearest point is projection point on segment
            chosen = segmentTangent * segmentProjection;
			chosen += ep0;
			return FixMath.F64Vec3.Distance(point, chosen);
		}
	}
}
