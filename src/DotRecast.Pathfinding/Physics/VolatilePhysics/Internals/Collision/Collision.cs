/*
 *  VolatilePhysics - A 2D Physics Library for Networked Games
 *  Copyright (c) 2015-2016 - Alexander Shoulson - http://ashoulson.com
 *
 *  This software is provided 'as-is', without any express or implied
 *  warranty. In no event will the authors be held liable for any damages
 *  arising from the use of this software.
 *  Permission is granted to anyone to use this software for any purpose,
 *  including commercial applications, and to alter it and redistribute it
 *  freely, subject to the following restrictions:
 *  
 *  1. The origin of this software must not be misrepresented; you must not
 *     claim that you wrote the original software. If you use this software
 *     in a product, an acknowledgment in the product documentation would be
 *     appreciated but is not required.
 *  2. Altered source versions must be plainly marked as such, and must not be
 *     misrepresented as being the original software.
 *  3. This notice may not be removed or altered from any source distribution.
*/

#if UNITY
using UnityEngine;
#endif

using FixMath.NET;

namespace Volatile
{
  internal static class Collision
  {
    #region Dispatch
    private delegate Manifold Test(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb);

    private readonly static Test[,] tests = new Test[,]
      {
        { __Circle_Circle, __Circle_Polygon },
        { __Polygon_Circle, __Polygon_Polygon}
      };

    internal static Manifold Dispatch(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      Test test = Collision.tests[(int)sa.Type, (int)sb.Type];
      return test(world, sa, sb);
    }

    private static Manifold __Circle_Circle(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Circle(world, (VoltCircle)sa, (VoltCircle)sb);
    }

    private static Manifold __Circle_Polygon(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Polygon(world, (VoltCircle)sa, (VoltPolygon)sb);
    }

    private static Manifold __Polygon_Circle(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Circle_Polygon(world, (VoltCircle)sb, (VoltPolygon)sa);
    }

    private static Manifold __Polygon_Polygon(
      VoltWorld world,
      VoltShape sa, 
      VoltShape sb)
    {
      return Polygon_Polygon(world, (VoltPolygon)sa, (VoltPolygon)sb);
    }
    #endregion

    #region Collision Tests
    private static Manifold Circle_Circle(
      VoltWorld world,
      VoltCircle circA,
      VoltCircle circB)
    {
      return 
        TestCircles(
          world,
          circA, 
          circB,
          circB.worldSpaceOrigin, 
          circB.radius);
    }

    private static Manifold Circle_Polygon(
      VoltWorld world,
      VoltCircle circ,
      VoltPolygon poly)
    {
      // Get the axis on the polygon closest to the circle's origin
      Fix64 penetration;
      int index =
        Collision.FindAxisMaxPenetration(
          circ.worldSpaceOrigin,
          circ.radius,
          poly,
          out penetration);

      if (index < 0)
        return null;

      VoltVector2 a, b;
      poly.GetEdge(index, out a, out b);
      Axis axis = poly.GetWorldAxis(index);

      // If the circle is past one of the two vertices, check it like
      // a circle-circle intersection where the vertex has radius 0
      Fix64 d = VoltMath.Cross(axis.Normal, circ.worldSpaceOrigin);
      if (d > VoltMath.Cross(axis.Normal, a))
        return Collision.TestCircles(world, circ, poly, a, Fix64.Zero);
      if (d < VoltMath.Cross(axis.Normal, b))
        return Collision.TestCircles(world, circ, poly, b, Fix64.Zero);

      // Build the collision Manifold
      Manifold manifold = world.AllocateManifold().Assign(world, circ, poly);
      VoltVector2 pos =
        circ.worldSpaceOrigin - (circ.radius + penetration / (Fix64)2) * axis.Normal;
      manifold.AddContact(pos, -axis.Normal, penetration);
      return manifold;
    }

    private static Manifold Polygon_Polygon(
      VoltWorld world,
      VoltPolygon polyA,
      VoltPolygon polyB)
    {
      Axis a1, a2;
      if (Collision.FindMinSepAxis(polyA, polyB, out a1) == false)
        return null;
      if (Collision.FindMinSepAxis(polyB, polyA, out a2) == false)
        return null;

      // We will use poly1's axis, so we may need to swap
      if (a2.Width > a1.Width)
      {
        VoltUtil.Swap(ref polyA, ref polyB);
        VoltUtil.Swap(ref a1, ref a2);
      }

      // Build the collision Manifold
      Manifold manifold = 
        world.AllocateManifold().Assign(world, polyA, polyB);
      Collision.FindVerts(polyA, polyB, a1.Normal, a1.Width, manifold);
      return manifold;
    }
    #endregion

    #region Common Tests and Queries
    /// <summary>
    /// Simple check for point-circle containment.
    /// </summary>
    internal static bool TestPointCircleSimple(
      VoltVector2 point,
      VoltVector2 origin,
      Fix64 radius)
    {
      VoltVector2 delta = origin - point;
      return delta.sqrMagnitude <= (radius * radius);
    }

    /// <summary>
    /// Simple check for two overlapping circles.
    /// </summary>
    internal static bool TestCircleCircleSimple(
      VoltVector2 originA,
      VoltVector2 originB,
      Fix64 radiusA,
      Fix64 radiusB)
    {
      Fix64 radiusTotal = radiusA + radiusB;
      return (originA - originB).sqrMagnitude <= (radiusTotal * radiusTotal);
    }

    /// <summary>
    /// Checks a ray against a circle with a given origin and square radius.
    /// </summary>
    internal static bool CircleRayCast(
      VoltShape shape,
      VoltVector2 shapeOrigin,
      Fix64 sqrRadius,
      ref VoltRayCast ray,
      ref VoltRayResult result)
    {
      VoltVector2 toOrigin = shapeOrigin - ray.origin;

      Fix64 slope = VoltVector2.Dot(toOrigin, ray.direction);
      if (slope < Fix64.Zero)
        return false;

      if (toOrigin.sqrMagnitude < sqrRadius)
      {
        result.SetContained(shape);
        return true;
      }

      Fix64 sqrSlope = slope * slope;
      Fix64 d = sqrRadius + sqrSlope - VoltVector2.Dot(toOrigin, toOrigin);
      if (d < Fix64.Zero)
        return false;

      Fix64 dist = slope - VoltMath.Sqrt(d);
      if (dist < Fix64.Zero || dist > ray.distance)
        return false;

      // N.B.: For historical raycasts this normal will be wrong!
      // Must be either transformed back to world or invalidated later.
      VoltVector2 normal = (dist * ray.direction - toOrigin).normalized;
      result.Set(shape, dist, normal);
      return true;
    }


    /// <summary>
    /// Returns the index of the nearest axis on the poly to a point.
    /// Outputs the minimum distance between the axis and the point.
    /// </summary>
    internal static int FindAxisShortestDistance(
      VoltVector2 point,
      Axis[] axes,
      out Fix64 minDistance)
    {
      int ix = 0;
      minDistance = Fix64.MaxValue;
      bool inside = true;

      for (int i = 0; i < axes.Length; i++)
      {
        Fix64 dot = VoltVector2.Dot(axes[i].Normal, point);
        Fix64 dist = axes[i].Width - dot;

        if (dist < Fix64.Zero)
          inside = false;

        if (dist < minDistance)
        {
          minDistance = dist;
          ix = i;
        }
      }

      if (inside == true)
      {
        minDistance = Fix64.Zero;
        ix = -1;
      }

      return ix;
    }

    /// <summary>
    /// Returns the index of the axis with the max circle penetration depth.
    /// Breaks out if a separating axis is found between the two shapes.
    /// Outputs the penetration depth of the circle in the axis (if any).
    /// </summary>
    internal static int FindAxisMaxPenetration(
      VoltVector2 origin,
      Fix64 radius,
      VoltPolygon poly,
      out Fix64 penetration)
    {
      int index = 0;
      int found = 0;
      penetration = Fix64.MinValue;

      for (int i = 0; i < poly.countWorld; i++)
      {
        Axis axis = poly.worldAxes[i];
        Fix64 dot = VoltVector2.Dot(axis.Normal, origin);
        Fix64 dist = dot - axis.Width - radius;

        if (dist > Fix64.Zero)
          return -1;

        if (dist > penetration)
        {
          penetration = dist;
          found = index;
        }

        index++;
      }

      return found;
    }
    #endregion

    #region Helpers
    /// <summary>
    /// Workhorse for circle-circle collisions, compares origin distance
    /// to the sum of the two circles' radii, returns a Manifold.
    /// </summary>
    /// 
    private static Manifold TestCircles(
      VoltWorld world,
      VoltCircle shapeA,
      VoltShape shapeB,
      VoltVector2 overrideBCenter, // For testing vertices in circles
      Fix64 overrideBRadius)
    {
      VoltVector2 r = overrideBCenter - shapeA.worldSpaceOrigin;
      Fix64 min = shapeA.radius + overrideBRadius;
      Fix64 distSq = r.sqrMagnitude;

      if (distSq >= min * min)
        return null;

      Fix64 dist = VoltMath.Sqrt(distSq);

      // 최소값을 지정하여 divide by zero 방지
      Fix64 distInv = Fix64.One / VoltMath.Max(dist, min / (Fix64)10);

      VoltVector2 pos =
        shapeA.worldSpaceOrigin +
        (Fix64.One / (Fix64)2 + distInv * (shapeA.radius - min / (Fix64)2)) * r;

      // Build the collision Manifold
      Manifold manifold = 
        world.AllocateManifold().Assign(world, shapeA, shapeB);
      manifold.AddContact(pos, distInv * r, dist - min);
      return manifold;
    }

    private static bool FindMinSepAxis(
      VoltPolygon poly1,
      VoltPolygon poly2,
      out Axis axis)
    {
      axis = new Axis(VoltVector2.zero, Fix64.MinValue);

      for (int i = 0; i < poly1.countWorld; i++)
      {
        Axis a = poly1.worldAxes[i];
        Fix64 min = Fix64.MaxValue;
        for (int j = 0; j < poly2.countWorld; j++)
        {
          VoltVector2 v = poly2.worldVertices[j];
          min = VoltMath.Min(min, VoltVector2.Dot(a.Normal, v));
        }
        min -= a.Width;

        if (min > Fix64.Zero)
          return false;
        if (min > axis.Width)
          axis = new Axis(a.Normal, min);
      }

      return true;
    }

    /// <summary>
    /// Add contacts for penetrating vertices. Note that this does not handle
    /// cases where an overlap was detected, but no vertices fall inside the
    /// opposing polygon (like a Star of David). For this we have a fallback.
    /// 
    /// See http://chipmunk-physics.googlecode.com/svn/trunk/src/cpCollision.c
    /// </summary>
    private static void FindVerts(
      VoltPolygon poly1,
      VoltPolygon poly2,
      VoltVector2 normal,
      Fix64 penetration,
      Manifold manifold)
    {
      bool found = false;

      for (int i = 0; i < poly1.countWorld; i++)
      {
        VoltVector2 vertex = poly1.worldVertices[i];
        if (poly2.ContainsPoint(vertex) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          found = true;
        }
      }

      for (int i = 0; i < poly2.countWorld; i++)
      {
        VoltVector2 vertex = poly2.worldVertices[i];
        if (poly1.ContainsPoint(vertex) == true)
        {
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
          found = true;
        }
      }

      // Fallback to check the degenerate "Star of David" case
      if (found == false)
        FindVertsFallback(poly1, poly2, normal, penetration, manifold);
    }

    /// <summary>
    /// A fallback for handling degenerate "Star of David" cases.
    /// </summary>
    private static void FindVertsFallback(
      VoltPolygon poly1,
      VoltPolygon poly2,
      VoltVector2 normal,
      Fix64 penetration,
      Manifold manifold)
    {
      for (int i = 0; i < poly1.countWorld; i++)
      {
        VoltVector2 vertex = poly1.worldVertices[i];
        if (poly2.ContainsPointPartial(vertex, normal) == true)
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
      }

      for (int i = 0; i < poly2.countWorld; i++)
      {
        VoltVector2 vertex = poly2.worldVertices[i];
        if (poly1.ContainsPointPartial(vertex, -normal) == true)
          if (manifold.AddContact(vertex, normal, penetration) == false)
            return;
      }
    }
    #endregion
  }
}
