using System.Numerics;
using System.Collections.Generic;

namespace SharpSteer2.Pathway
{
    /// <summary>
    /// A path consisting of a series of gates which must be passed through
    /// </summary>
    public class GatewayPathway
        : IPathway
    {
        public PolylinePathway Centerline
        {
            get
            {
                return _trianglePathway.Centerline;
            }
        }

        private readonly TrianglePathway _trianglePathway;
        public TrianglePathway TrianglePathway
        {
            get
            {
                return _trianglePathway;
            }
        }

        public GatewayPathway(IEnumerable<Gateway> gateways, bool cyclic = false)
        {
            List<TrianglePathway.Triangle> triangles = new List<TrianglePathway.Triangle>();

            bool first = true;
            Gateway previous = default(Gateway);
            var previousNormalized = FixMath.F64Vec3.Zero;
            foreach (var gateway in gateways)
            {
                var n = FixMath.F64Vec3.NormalizeFast(gateway.B - gateway.A);

                if (!first)
                {
                    if (FixMath.F64Vec3.Dot(n, previousNormalized) < 0)
                    {
                        triangles.Add(new TrianglePathway.Triangle(previous.A, previous.B, gateway.A));
                        triangles.Add(new TrianglePathway.Triangle(previous.A, gateway.A, gateway.B));
                    }
                    else
                    {
                        triangles.Add(new TrianglePathway.Triangle(previous.A, previous.B, gateway.A));
                        triangles.Add(new TrianglePathway.Triangle(previous.B, gateway.A, gateway.B));
                    }
                }
                first = false;

                previousNormalized = n;
                previous = gateway;
            }

            _trianglePathway = new TrianglePathway(triangles, cyclic);

        }

        public struct Gateway
        {
            public readonly FixMath.F64Vec3 A;
            public readonly FixMath.F64Vec3 B;

            public Gateway(FixMath.F64Vec3 a, FixMath.F64Vec3 b)
                : this()
            {
                A = a;
                B = b;
            }
        }

        public FixMath.F64Vec3 MapPointToPath(FixMath.F64Vec3 point, out FixMath.F64Vec3 tangent, out FixMath.F64 outside)
        {
            return _trianglePathway.MapPointToPath(point, out tangent, out outside);
        }

        public FixMath.F64Vec3 MapPathDistanceToPoint(FixMath.F64 pathDistance)
        {
            return _trianglePathway.MapPathDistanceToPoint(pathDistance);
        }

        public FixMath.F64 MapPointToPathDistance(FixMath.F64Vec3 point)
        {
            return _trianglePathway.MapPointToPathDistance(point);
        }
    }
}
