

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
                drawEdge(edge, edge.get_isConstrained() ? UnityEngine.Color.red : UnityEngine.Color.blue);
            }
        }
    }
}