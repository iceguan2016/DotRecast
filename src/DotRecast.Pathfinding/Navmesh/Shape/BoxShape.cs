using System.Collections;
using System.Collections.Generic;

using Game.Utils;
using UnityEngine;

namespace Navmesh.Shape
{
    public class BoxShape
    {
        public Vector3      Position = Vector3.zero;
        public Quaternion   Rotation = Quaternion.identity;
        public Vector3      HalfExtent = Vector3.one;

        public Navmesh.FConvexShape GetConvexShape()
        {
            Vector3[] Vertices = new Vector3[] {
                Position + Rotation * new Vector3( HalfExtent.x, -HalfExtent.y,  HalfExtent.z), // 0
                Position + Rotation * new Vector3(-HalfExtent.x, -HalfExtent.y,  HalfExtent.z), // 1
                Position + Rotation * new Vector3(-HalfExtent.x, -HalfExtent.y, -HalfExtent.z), // 2
                Position + Rotation * new Vector3( HalfExtent.x, -HalfExtent.y, -HalfExtent.z), // 3
        };

            return new Navmesh.FConvexShape(Vertices);
        }

        public bool IsContainPoint(Vector3 p)
        {
            var local = Quaternion.Inverse(Rotation) * (p - Position);
            return  Mathf.Abs(local.x) <= HalfExtent.x &&
                    Mathf.Abs(local.y) <= HalfExtent.y &&
                    Mathf.Abs(local.z) <= HalfExtent.z;
        }

        public void DrawGizmos()
        {
            Debug.DrawObbCube(Position, Rotation, HalfExtent, Color.green);
        }
    }
}
