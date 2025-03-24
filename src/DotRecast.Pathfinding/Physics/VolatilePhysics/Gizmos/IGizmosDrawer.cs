using System;
using System.Collections.Generic;
using System.Text;

namespace Volatile
{
    public interface IGizmosDrawer
    {
        void DrawLine(VoltVector2 a, VoltVector2 b, Color c, float lineWidth = 1.0f);

        void DrawArrow(VoltVector2 start, VoltVector2 end, VoltVector2 arrowSize, float lineWidth, Color c);

        void DrawCube(VoltVector2 p, VoltVector2 size, Color c);

        void DrawCircle(VoltVector2 p, float r, Color c);

        void DrawTriangle(VoltVector2 v0, VoltVector2 v1, VoltVector2 v2, Color c);

        void DrawSolidCube(VoltVector2 p, FixMath.NET.Fix64 angle, VoltVector2 size, Color c);
    }
}
