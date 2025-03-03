
using UnityEngine;
using DotRecast.Detour;
using System.Collections.Generic;
using DotRecast.Core;
using hxDaedalus.iterators;
using System;
using Navmesh.Utils;

namespace Navmesh
{
    public class FNavmeshQuery
    {
        public hxDaedalus.data.Mesh NavMesh { get; private set; }

        private int MaxNodes = 1024;

        public bool Initialize(hxDaedalus.data.Mesh InNavmesh, int InMaxNodes)
        {
            NavMesh     = InNavmesh;
            MaxNodes    = InMaxNodes;
            return true;
        }

        /// Finds the polygon nearest to the specified center point.
        /// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
        /// 
        ///  @param[in]		center		The center of the search box. [(x, y, z)]
        ///  @param[in]		halfExtents	The search distance along each axis. [(x, y, z)]
        ///  @param[in]		filter		The polygon filter to apply to the query.
        ///  @param[out]	nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
        ///  @param[out]	nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
        ///  @param[out]	isOverPoly 	Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
        /// @returns The status flags for the query.
        public DtStatus FindNearestPoly(UnityEngine.Vector3 center, UnityEngine.Vector3 halfExtents, IDtQueryFilter filter,
            out long nearestRef, out UnityEngine.Vector3 nearestPt, out bool isOverPoly)
        {
            nearestRef = 0;
            nearestPt = center;
            isOverPoly = false;

            hxDaedalus.data.math.Intersection loc = hxDaedalus.data.math.Geom2D.locatePosition(center.x, center.z, NavMesh);
            switch (loc._hx_index)
            {
                case 0:
                    {
                        global::hxDaedalus.data.Vertex vertex = (loc as global::hxDaedalus.data.math.Intersection_EVertex).vertex;
                        var pos = vertex.get_pos();
                        var edge = vertex.get_edge();
                        var fromFace = edge.get_leftFace();
                        nearestRef = fromFace.get_id();
                        break;
                    }
                case 1:
                    {
                        global::hxDaedalus.data.Edge edge = (loc as global::hxDaedalus.data.math.Intersection_EEdge).edge;
                        var fromFace = edge.get_leftFace();
                        nearestRef = fromFace.get_id();
                        break;
                    }
                case 2:
                    {
                        global::hxDaedalus.data.Face face = (loc as global::hxDaedalus.data.math.Intersection_EFace).face;
                        nearestRef = face.get_id();
                        break;
                    }
                case 3:
                    {
                        break;
                    }
            }

            return DtStatus.DT_SUCCESS;
        }

        /// @par
        ///
        /// If the @p segmentRefs parameter is provided, then all polygon segments will be returned.
        /// Otherwise only the wall segments are returned.
        ///
        /// A segment that is normally a portal will be included in the result set as a
        /// wall if the @p filter results in the neighbor polygon becoomming impassable.
        ///
        /// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the
        /// maximum segments per polygon of the source navigation mesh.
        ///
        /// Returns the segments for the specified polygon, optionally including portals.
        /// @param[in] ref The reference id of the polygon.
        /// @param[in] filter The polygon filter to apply to the query.
        /// @param[out] segmentVerts The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
        /// @param[out] segmentRefs The reference ids of each segment's neighbor polygon.
        /// Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
        /// @param[out] segmentCount The number of segments returned.
        /// @param[in] maxSegments The maximum number of segments the result arrays can hold.
        /// @returns The status flags for the query.
        public DtStatus GetPolyWallSegments(long refs, bool storePortals, IDtQueryFilter filter,
            ref List<RcSegmentVert> segmentVerts, ref List<long> segmentRefs)
        {
            segmentVerts.Clear();
            segmentRefs.Clear();

            if (null == filter)
            {
                return DtStatus.DT_FAILURE | DtStatus.DT_INVALID_PARAM;
            }

            // 
            var face = NavMesh._edges[(int)refs] as hxDaedalus.data.Face;
            var iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
            iterEdge.set_fromFace(face);

            while (true)
            {
                var innerEdge = iterEdge.next();
                if (innerEdge == null)
                {
                    break;
                }

                // Wall segment
                if (innerEdge.get_isConstrained())
                {
                    var seg = new RcSegmentVert();
                    var p0 = innerEdge.get_originVertex().get_pos();
                    var p1 = innerEdge.get_destinationVertex().get_pos();
                    seg.vmin = p0.ToRecastVec3();
                    seg.vmax = p1.ToRecastVec3();
                    segmentVerts.Add(seg);
                    segmentRefs.Add(0L);
                }
            }

            return DtStatus.DT_SUCCESS;
        }
    }

}