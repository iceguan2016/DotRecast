

using System.Collections.Generic;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Util;

namespace Pathfinding.Triangulation.View
{
    public class SimpleView
    {
        DrawInterface _draw = null;

        public SimpleView(DrawInterface draw)
        {
            _draw = draw;
        }

        public void drawVertex(Vertex vertex)
        {
            var p = vertex.get_pos();
            var dp = new UnityEngine.Vector3(p.X.Float, _draw.GetMapHeight(), p.Y.Float);

            _draw.DrawCircle(dp, 0.1f, UnityEngine.Color.red);
        }

        public void drawEdge(Edge edge, UnityEngine.Color c, float lineWidth = 1.0f)
        {
            var v0 = edge.get_originVertex();
            var v1 = edge.get_destinationVertex();

            var p0 = v0.get_pos();
            var p1 = v1.get_pos();

            var dp0 = new UnityEngine.Vector3(p0.X.Float, _draw.GetMapHeight(), p0.Y.Float);
            var dp1 = new UnityEngine.Vector3(p1.X.Float, _draw.GetMapHeight(), p1.Y.Float);

            _draw.DrawLine(dp0, dp1, c, lineWidth);
        }

        public void drawFace(Face face, UnityEngine.Color c)
        {
            var verts = new UnityEngine.Vector3[3];
            var index = 0;

            var iterEdge = new FromFaceToInnerEdges();
            iterEdge.set_fromFace(face);
            while (true)
            {
                var innerEdge = iterEdge.next();
                if (innerEdge == null)
                {
                    break;
                }

                var p = innerEdge.get_originVertex().get_pos();
                var dp = new UnityEngine.Vector3(p.X.Float, _draw.GetMapHeight(), p.Y.Float);
                verts[index] = dp;
                ++index;
            }

            if (index == 3)
            {
                _draw.DrawTriangle(verts[0], verts[1], verts[2], c);
            }
        }

        public void drawMesh(Mesh mesh)
        {
            var (vertices, edges) = mesh.getVerticesAndEdges();

            for (var i = 0; i < vertices.Count; ++i)
            {
                drawVertex(vertices[i]);
            }

            for (var i = 0; i < edges.Count; ++i)
            {
                var edge = edges[i];
                
                drawEdge(edge, edge.get_isConstrained() ? (edge._isReversed? UnityEngine.Color.green : UnityEngine.Color.red) : UnityEngine.Color.blue);
            }
        }

        public void drawObstacleFaces(Mesh mesh, int maxDepth, bool expand = false)
        {
            var obstacles = mesh._objects;

            var queue = new Queue<Face>();
            var visitedFaces = new HashSet<Face>();
            var depthMap = new Dictionary<Face, int>();
            for (var i = 0; i < obstacles.Count; ++i)
            {
                var shape = obstacles[i]._constraintShape;
                var segments = shape.segments;
                for (var j = 0; j < segments.Count; ++j)
                {
                    var edges = segments[j]._edges;
                    for (var k = 0; k < edges.Count; ++k)
                    {
                        var face = edges[k]._isReversed? edges[k].get_leftFace() : edges[k].get_rightFace();
                        if (visitedFaces.Add(face))
                        {
                            queue.Enqueue(face);
                            depthMap.Add(face, 0);
                        }
                    }
                }
            }

            if (expand)
            {
                var iterEdge = new FromFaceToInnerEdges();
                while (queue.Count > 0)
                {
                    var face = queue.Dequeue();
                    var depth = depthMap[face];
                    if (depth >= maxDepth) continue;

                    // 遍历邻接face
                    Edge innerEdge = null;
                    iterEdge.set_fromFace(face);
                    while ((innerEdge = iterEdge.next()) != null)
                    {
                        if (innerEdge._isConstrained)
                            continue;
                        var neighbourFace = innerEdge.get_rightFace();
                        if (visitedFaces.Add(neighbourFace))
                        {
                            queue.Enqueue(neighbourFace);
                            depthMap.Add(neighbourFace, depth+1);
                        }
                    }
                }
            }

            // draw all faces
            var color = new UnityEngine.Color(1, 0, 0, 0.3f);
            foreach (var face in visitedFaces)
            { 
                drawFace(face, color);
            }
        }
    }
}