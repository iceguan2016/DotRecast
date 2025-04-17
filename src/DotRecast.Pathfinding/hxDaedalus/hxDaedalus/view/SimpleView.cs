using hxDaedalus.data;
using Pathfinding.Util;

namespace hxDaedalus.view
{
    public class SimpleView
    {
        IDrawInterface _draw = null;

        public SimpleView(IDrawInterface draw) 
        {
            _draw = draw;
        }

        public void drawVertex(Vertex vertex)
	    {
		    var p = vertex.get_pos();
            var dp = new UnityEngine.Vector3((float)p.x, _draw.TerrainHeight, (float)p.y);

            _draw.DrawCircle(dp, 0.1f, UnityEngine.Color.red);
        }

        public void drawEdge(Edge edge, UnityEngine.Color c, float lineWidth=1.0f)
        { 
            var v0 = edge.get_originVertex();
            var v1 = edge.get_destinationVertex();

            var p0 = v0.get_pos();
            var p1 = v1.get_pos();

            var dp0 = new UnityEngine.Vector3((float)p0.x, _draw.TerrainHeight, (float)p0.y);
            var dp1 = new UnityEngine.Vector3((float)p1.x, _draw.TerrainHeight, (float)p1.y);

            _draw.DrawLine(dp0, dp1, c, lineWidth);
        }

        public void drawFace(Face face, UnityEngine.Color c)
        {
            var verts = new UnityEngine.Vector3[3];
            var index = 0;

            var iterEdge = new hxDaedalus.iterators.FromFaceToInnerEdges();
            iterEdge.set_fromFace(face);
            while (true)
            {
                var innerEdge = iterEdge.next();
                if (innerEdge == null)
                {
                    break;
                }

                var p = innerEdge.get_originVertex().get_pos();
                var dp = new UnityEngine.Vector3((float)p.x, _draw.TerrainHeight, (float)p.y);
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
            var all = mesh.getVerticesAndEdges();
            var vertices = HxReflect.field(all, "vertices") as HxArray<object>;
            var edges = HxReflect.field(all, "edges") as HxArray<object>;
            //var vertices = mesh._vertices;
            //var edges = mesh._edges;

            for (var i = 0; i < vertices.length; ++i)
            {
                drawVertex((Vertex)vertices[i]);
            }

            for (var i = 0; i < edges.length; ++i)
            {
                var edge = (Edge)edges[i];
                drawEdge(edge, edge.get_isConstrained() ? UnityEngine.Color.red : UnityEngine.Color.blue);
            }
        }
    }
}
