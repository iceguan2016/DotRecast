using UnityEngine;

namespace Game.Utils
{
    public interface DrawInterface
    {
        void DrawLine(Vector3 a, Vector3 b, Color c);
    }

    public static class Debug
    {
        public static DrawInterface drawInterface = null;

        public static void Assert(bool condition, string desc="")
        { 
        }

        public static void Log(string content)
        { 

        }

        public static void LogWarning(string content)
        {

        }

        public static void LogError(string content)
        {

        }

        public static void DrawLine(Vector3 a, Vector3 b, Color c, float duration=0.0f)
        { 
            if (drawInterface != null)
            {
                drawInterface.DrawLine(a, b, c);
            }
        }

        public static void DrawRay(Vector3 origin, Vector3 direction, Color c, float duration=0.0f)
        {
            if (drawInterface != null)
            {
                var end = origin + direction * 30.0f;
                drawInterface.DrawLine(origin, end, c);
            }
        }

        public static void DrawCube(Vector3 center, Vector3 extent, Color c, float duration = 0.0f)
        {
            Vector3[] Vertices = new Vector3[] {
                center + new Vector3( extent.x, -extent.y,  extent.z), // 0
                center + new Vector3(-extent.x, -extent.y,  extent.z), // 1
                center + new Vector3(-extent.x, -extent.y, -extent.z), // 2
                center + new Vector3( extent.x, -extent.y, -extent.z), // 3

                center + new Vector3( extent.x,  extent.y,  extent.z), // 4
                center + new Vector3(-extent.x,  extent.y,  extent.z), // 5
                center + new Vector3(-extent.x,  extent.y, -extent.z), // 6
                center + new Vector3( extent.x,  extent.y, -extent.z), // 7
            };

            int[] Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
            };

            for (int i = 0; i < Indices.Length; i += 2)
            {
                Debug.DrawLine(Vertices[Indices[i]], Vertices[Indices[i + 1]], c);
            }
        }

        public static void DrawObbCube(Vector3 InPosition, Quaternion InRotation, Vector3 InHalfExtent, Color InColor)
        {
            Vector3[] Vertices = new Vector3[] {
                InPosition + InRotation * new Vector3( InHalfExtent.x, -InHalfExtent.y,  InHalfExtent.z), // 0
                InPosition + InRotation * new Vector3(-InHalfExtent.x, -InHalfExtent.y,  InHalfExtent.z), // 1
                InPosition + InRotation * new Vector3(-InHalfExtent.x, -InHalfExtent.y, -InHalfExtent.z), // 2
                InPosition + InRotation * new Vector3( InHalfExtent.x, -InHalfExtent.y, -InHalfExtent.z), // 3

                InPosition + InRotation * new Vector3( InHalfExtent.x,  InHalfExtent.y,  InHalfExtent.z), // 4
                InPosition + InRotation * new Vector3(-InHalfExtent.x,  InHalfExtent.y,  InHalfExtent.z), // 5
                InPosition + InRotation * new Vector3(-InHalfExtent.x,  InHalfExtent.y, -InHalfExtent.z), // 6
                InPosition + InRotation * new Vector3( InHalfExtent.x,  InHalfExtent.y, -InHalfExtent.z), // 7
            };

            int[] Indices = new int[] {
                0, 1, 1, 2, 2, 3, 3, 0,
                4, 5, 5, 6, 6, 7, 7, 4,
                0, 4, 1, 5, 2, 6, 3, 7
            };

            for (int i = 0; i < Indices.Length; i += 2)
            {
                Debug.DrawLine(Vertices[Indices[i]], Vertices[Indices[i + 1]], InColor);
            }
        }

        public static void DrawSphere(Vector3 center, float radius, Color c, float duration = 0.0f)
        { 
        }
    }
}
