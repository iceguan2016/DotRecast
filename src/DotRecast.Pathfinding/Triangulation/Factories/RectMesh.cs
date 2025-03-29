
using System.Collections.Generic;
using Pathfinding.Triangulation.Data;

namespace Pathfinding.Triangulation.Factories
{
    public class RectMesh
    {
        public static Mesh buildRectangle(FixMath.F64 x, FixMath.F64 y, FixMath.F64 width, FixMath.F64 height)
        {
            /*
               TL
                 ----+-----+ TR
            \   |    /|
            \   |   / |
            \   |  /  |
            \   | /   |
            \   |/    |
            \   +-----+ BR
            \  BL     \
            \----------
            */

            var vTL = new Vertex();
            var vTR = new Vertex();
            var vBR = new Vertex();
            var vBL = new Vertex();

            var eTL_TR = new Edge();
            var eTR_TL = new Edge();
            var eTR_BR = new Edge();
            var eBR_TR = new Edge();
            var eBR_BL = new Edge();
            var eBL_BR = new Edge();
            var eBL_TL = new Edge();
            var eTL_BL = new Edge();
            var eTR_BL = new Edge();
            var eBL_TR = new Edge();
            var eTL_BR = new Edge();
            var eBR_TL = new Edge();

            var fTL_BL_TR = new Face();
            var fTR_BL_BR = new Face();
            var fTL_BR_BL = new Face();
            var fTL_TR_BR = new Face();

            var boundShape = new ConstraintShape();
            var segTop = new ConstraintSegment();
            var segRight = new ConstraintSegment();
            var segBot = new ConstraintSegment();
            var segLeft = new ConstraintSegment();

            var mesh = new Mesh(width, height);

            //

            var offset = Constants.EPSILON * 1000;
            vTL._pos = new FixMath.F64Vec2(x - offset, y - offset);
            vTR._pos = new FixMath.F64Vec2(x + width + offset, y - offset);
            vBR._pos = new FixMath.F64Vec2(x + width + offset, y + height + offset);
            vBL._pos = new FixMath.F64Vec2(x - offset, y + height + offset);

            vTL.setDatas(eTL_TR);
            vTR.setDatas(eTR_BR);
            vBR.setDatas(eBR_BL);
            vBL.setDatas(eBL_TL);

            eTL_TR.setDatas(vTL, eTR_TL, eTR_BR, fTL_TR_BR, true, true);
            eTR_TL.setDatas(vTR, eTL_TR, eTL_BL, fTL_BL_TR, true, true);
            eTR_BR.setDatas(vTR, eBR_TR, eBR_TL, fTL_TR_BR, true, true);
            eBR_TR.setDatas(vBR, eTR_BR, eTR_BL, fTR_BL_BR, true, true);
            eBR_BL.setDatas(vBR, eBL_BR, eBL_TL, fTL_BR_BL, true, true);
            eBL_BR.setDatas(vBL, eBR_BL, eBR_TR, fTR_BL_BR, true, true);
            eBL_TL.setDatas(vBL, eTL_BL, eTL_BR, fTL_BR_BL, true, true);
            eTL_BL.setDatas(vTL, eBL_TL, eBL_TR, fTL_BL_TR, true, true);
            eTR_BL.setDatas(vTR, eBL_TR, eBL_BR, fTR_BL_BR, true, false);  // diagonal edge  
            eBL_TR.setDatas(vBL, eTR_BL, eTR_TL, fTL_BL_TR, true, false);  // diagonal edge  
            eTL_BR.setDatas(vTL, eBR_TL, eBR_BL, fTL_BR_BL, false, false);  // imaginary edge  
            eBR_TL.setDatas(vBR, eTL_BR, eTL_TR, fTL_TR_BR, false, false);  // imaginary edge  

            fTL_BL_TR.setDatas(eBL_TR);
            fTR_BL_BR.setDatas(eTR_BL);
            fTL_BR_BL.setDatas(eBR_BL, false);
            fTL_TR_BR.setDatas(eTR_BR, false);

            // constraint relations datas
            vTL._fromConstraintSegments = new List<ConstraintSegment> { segTop, segLeft };
            vTR._fromConstraintSegments = new List<ConstraintSegment> { segTop, segRight };
            vBR._fromConstraintSegments = new List<ConstraintSegment> { segRight, segBot };
            vBL._fromConstraintSegments = new List<ConstraintSegment> { segBot, segLeft };

            eTL_TR.fromConstraintSegments.Add(segTop);
            eTR_TL.fromConstraintSegments.Add(segTop);
            eTR_BR.fromConstraintSegments.Add(segRight);
            eBR_TR.fromConstraintSegments.Add(segRight);
            eBR_BL.fromConstraintSegments.Add(segBot);
            eBL_BR.fromConstraintSegments.Add(segBot);
            eBL_TL.fromConstraintSegments.Add(segLeft);
            eTL_BL.fromConstraintSegments.Add(segLeft);

            segTop._edges.Add(eTL_TR);
            segRight._edges.Add(eTR_BR);
            segBot._edges.Add(eBR_BL);
            segLeft._edges.Add(eBL_TL);
            segTop.fromShape = boundShape;
            segRight.fromShape = boundShape;
            segBot.fromShape = boundShape;
            segLeft.fromShape = boundShape;

            boundShape.segments.AddRange(new ConstraintSegment[] { segTop, segRight, segBot, segLeft });
            mesh._vertices.AddRange(new Vertex[] { vTL, vTR, vBR, vBL });
            mesh._edges.AddRange(new Edge[] { eTL_TR, eTR_TL, eTR_BR, eBR_TR, eBR_BL, eBL_BR, eBL_TL, eTL_BL, eTR_BL, eBL_TR, eTL_BR, eBR_TL });
            mesh._faces.AddRange(new Face[] { fTL_BL_TR, fTR_BL_BR, fTL_BR_BL, fTL_TR_BR });

            mesh._constraintShapes.Add(boundShape);
            var securityRect = new List<FixMath.F64>();
            securityRect.AddRange(new FixMath.F64[] { FixMath.F64.Zero, FixMath.F64.Zero, width, FixMath.F64.Zero });
            securityRect.AddRange(new FixMath.F64[] { width, FixMath.F64.Zero, width, height });
            securityRect.AddRange(new FixMath.F64[] { width, height, FixMath.F64.Zero, height });
            securityRect.AddRange(new FixMath.F64[] { FixMath.F64.Zero, height, FixMath.F64.Zero, FixMath.F64.Zero });


            mesh.set_clipping(false);
            mesh.insertConstraintShape(securityRect);
            mesh.set_clipping(true);

            return mesh;
        }
    }
}
