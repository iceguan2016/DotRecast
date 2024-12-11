using System;
using System.Security.Cryptography;
using hxDaedalus.data;

namespace hxDaedalus.view
{
    public class SimpleView
    {
        Game.Utils.DrawInterface _draw = null;

        public SimpleView(Game.Utils.DrawInterface draw) 
        {
            _draw = draw;
        }

        public void drawVertex(Vertex vertex)
	    {
		    var p = vertex.get_pos();
            var dp = new UnityEngine.Vector3((float)p.x, _draw.GetMapHeight(), (float)p.y);

            _draw.DrawCircle(dp, 0.1f, UnityEngine.Color.red);
        }

        public void drawEdge(Edge edge, UnityEngine.Color c, float lineWidth=1.0f)
        { 
            var v0 = edge.get_originVertex();
            var v1 = edge.get_destinationVertex();

            var p0 = v0.get_pos();
            var p1 = v1.get_pos();

            var dp0 = new UnityEngine.Vector3((float)p0.x, _draw.GetMapHeight(), (float)p0.y);
            var dp1 = new UnityEngine.Vector3((float)p1.x, _draw.GetMapHeight(), (float)p1.y);

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
                var dp = new UnityEngine.Vector3((float)p.x, _draw.GetMapHeight(), (float)p.y);
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
            // var all = mesh.getVerticesAndEdges();
            // var vertices = HxReflect.field(all, "_vertices");
		    
            for (var i=0; i<mesh._vertices.length; ++i)
            {
                drawVertex((Vertex)mesh._vertices[i]);
            }

            for (var i = 0; i < mesh._edges.length; ++i)
            {
                var edge = (Edge)mesh._edges[i];
                drawEdge(edge, edge.get_isConstrained() ? UnityEngine.Color.red : UnityEngine.Color.blue);
            }
        }
    }
}
