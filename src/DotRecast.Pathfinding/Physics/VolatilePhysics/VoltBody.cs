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

using FixMath.NET;
using System;

#if UNITY
using UnityEngine;
#endif

namespace Volatile
{
  public enum VoltBodyType
  {
    Static,
    Dynamic,
    Invalid,
  }

  public delegate bool VoltBodyFilter(VoltBody body);
  public delegate bool VoltCollisionFilter(VoltBody bodyA, VoltBody bodyB);

  public class VoltBody
    : IVoltPoolable<VoltBody>
    , IIndexedValue
  {
    #region Interface
    IVoltPool<VoltBody> IVoltPoolable<VoltBody>.Pool { get; set; }
    void IVoltPoolable<VoltBody>.Reset() { this.Reset(); }
    int IIndexedValue.Index { get; set; }
    #endregion

    /// <summary>
    /// A predefined filter that disallows collisions between dynamic bodies.
    /// </summary>
    public static bool DisallowDynamic(VoltBody a, VoltBody b)
    {
      return
        (a != null) &&
        (b != null) &&
        (a.IsStatic || b.IsStatic);
    }

    #region History
    /// <summary>
    /// Tries to get a reference frame for a given number of ticks behind 
    /// the current tick. Returns true if a value was found, false if a
    /// value was not found (in which case we clamp to the nearest).
    /// </summary>
    public bool TryGetSpace(
      int ticksBehind,
      out VoltVector2 position,
      out VoltVector2 facing)
    {
      if (ticksBehind < 0)
        throw new ArgumentOutOfRangeException("ticksBehind");

      if (ticksBehind == 0)
      {
        position = this.Position;
        facing = this.Facing;
        return true;
      }

      if (this.history == null)
      {
        position = this.Position;
        facing = this.Facing;
        return false;
      }

      HistoryRecord record;
      bool found = this.history.TryGet(ticksBehind - 1, out record);
      position = record.position;
      facing = record.facing;
      return found;
    }

    /// <summary>
    /// Initializes the buffer for storing past body states/spaces.
    /// </summary>
    internal void AssignHistory(HistoryBuffer history)
    {
      VoltDebug.Assert(this.IsStatic == false);
      this.history = history;
    }

    /// <summary>
    /// Stores a snapshot of this body's current state/space to a tick.
    /// </summary>
    private void StoreState()
    {
      if (this.history != null)
        this.history.Store(this.currentState);
    }

    /// <summary>
    /// Retrieves a snapshot of the body's state/space at a tick.
    /// Logs an error and defaults to the current state if it can't be found.
    /// </summary>
    private HistoryRecord GetState(int ticksBehind)
    {
      if ((ticksBehind == 0) || (this.history == null))
        return this.currentState;

      HistoryRecord output;
      this.history.TryGet(ticksBehind - 1, out output);
      return output;
    }
    #endregion

    public static bool Filter(VoltBody body, VoltBodyFilter filter)
    {
      return ((filter == null) || (filter.Invoke(body) == true));
    }

    /// <summary>
    /// Static objects are considered to have infinite mass and cannot move.
    /// </summary>
    public bool IsStatic
    {
      get
      {
        if (this.BodyType == VoltBodyType.Invalid)
          throw new InvalidOperationException();
        return this.BodyType == VoltBodyType.Static;
      }
    }

    /// <summary>
    /// If we're doing historical queries or tests, the body may have since
    /// been removed from the world.
    /// </summary>
    public bool IsInWorld { get { return this.World != null; } }

    // Some basic properties are stored in an internal mutable
    // record to avoid code redundancy when performing conversions
    public VoltVector2 Position
    {
      get { return this.currentState.position; }
      private set { this.currentState.position = value; }
    }

    public VoltVector2 Facing
    {
      get { return this.currentState.facing; }
      private set { this.currentState.facing = value; }
    }

    public VoltAABB AABB
    {
      get { return this.currentState.aabb; }
      private set { this.currentState.aabb = value; }
    }

#if DEBUG
    internal bool IsInitialized { get; set; }
#endif

    /// <summary>
    /// For attaching arbitrary data to this body.
    /// </summary>
    public object UserData { get; set; }

    public VoltWorld World { get; private set; }
    public VoltBodyType BodyType { get; private set; }
    public VoltCollisionFilter CollisionFilter { private get; set; }

    /// <summary>
    /// Current angle in radians.
    /// </summary>
    public Fix64 Angle { get; private set; }

    public VoltVector2 LinearVelocity { get; set; }
    public Fix64 AngularVelocity { get; set; }

    public VoltVector2 Force { get; private set; }
    public Fix64 Torque { get; private set; }

    public Fix64 Mass { get; private set; }
    public Fix64 Inertia { get; private set; }
    public Fix64 InvMass { get; private set; }
    public Fix64 InvInertia { get; private set; }

    public VoltVector2 BiasVelocity { get; private set; }
    public Fix64 BiasRotation { get; private set; }

    // Used for broadphase structures
    internal int ProxyId { get; set; }

    internal VoltShape[] shapes;
    internal int shapeCount;

    private HistoryBuffer history;
    private HistoryRecord currentState;

    #region Manipulation
    public void AddTorque(Fix64 torque)
    {
      this.Torque += torque;
    }

    public void AddForce(VoltVector2 force)
    {
      this.Force += force;
    }

    public void AddForce(VoltVector2 force, VoltVector2 point)
    {
      this.Force += force;
      this.Torque += VoltMath.Cross(this.Position - point, force);
    }

    public void Set(VoltVector2 position, Fix64 radians)
    {
      this.Position = position;
      this.Angle = radians;
      this.Facing = VoltMath.Polar(radians);
      this.OnPositionUpdated();
    }
    
    public void SetForce(VoltVector2 force, Fix64 torque, VoltVector2 biasVelocity, Fix64 biasRotation)
    {
      this.Force = force;
      this.Torque = torque;
      this.BiasVelocity = biasVelocity;
      this.BiasRotation = biasRotation;
    }
    
    #endregion

    #region Tests
    /// <summary>
    /// Checks if an AABB overlaps with our AABB.
    /// </summary>
    internal bool QueryAABBOnly(
      VoltAABB worldBounds,
      int ticksBehind)
    {
      HistoryRecord record = this.GetState(ticksBehind);

      // AABB check done in world space (because it keeps changing)
      return record.aabb.Intersect(worldBounds);
    }

    /// <summary>
    /// Checks if a point is contained in this body. 
    /// Begins with AABB checks unless bypassed.
    /// </summary>
    internal bool QueryPoint(
      VoltVector2 point,
      int ticksBehind,
      bool bypassAABB = false)
    {
      HistoryRecord record = this.GetState(ticksBehind);

      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (record.aabb.QueryPoint(point) == false)
          return false;

      // Actual query on shapes done in body space
      VoltVector2 bodySpacePoint = record.WorldToBodyPoint(point);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].QueryPoint(bodySpacePoint))
          return true;
      return false;
    }

    /// <summary>
    /// Checks if a circle overlaps with this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool QueryCircle(
      VoltVector2 origin,
      Fix64 radius,
      int ticksBehind,
      bool bypassAABB = false)
    {
      HistoryRecord record = this.GetState(ticksBehind);

      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (record.aabb.QueryCircleApprox(origin, radius) == false)
          return false;

      // Actual query on shapes done in body space
      VoltVector2 bodySpaceOrigin = record.WorldToBodyPoint(origin);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].QueryCircle(bodySpaceOrigin, radius))
          return true;
      return false;
    }

    /// <summary>
    /// Performs a ray cast check on this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool RayCast(
      ref VoltRayCast ray,
      ref VoltRayResult result,
      int ticksBehind,
      bool bypassAABB = false)
    {
      HistoryRecord record = this.GetState(ticksBehind);

      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (record.aabb.RayCast(ref ray) == false)
          return false;

      // Actual tests on shapes done in body space
      VoltRayCast bodySpaceRay = record.WorldToBodyRay(ref ray);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].RayCast(ref bodySpaceRay, ref result))
          if (result.IsContained)
            return true;

      // We need to convert the results back to world space to be any use
      // (Doesn't matter if we were contained since there will be no normal)
      if (result.Body == this)
        result.normal = record.BodyToWorldDirection(result.normal);
      return result.IsValid;
    }

    /// <summary>
    /// Performs a circle cast check on this body. 
    /// Begins with AABB checks.
    /// </summary>
    internal bool CircleCast(
      ref VoltRayCast ray,
      Fix64 radius,
      ref VoltRayResult result,
      int ticksBehind,
      bool bypassAABB = false)
    {
      HistoryRecord record = this.GetState(ticksBehind);

      // AABB check done in world space (because it keeps changing)
      if (bypassAABB == false)
        if (record.aabb.CircleCastApprox(ref ray, radius) == false)
          return false;

      // Actual tests on shapes done in body space
      VoltRayCast bodySpaceRay = record.WorldToBodyRay(ref ray);
      for (int i = 0; i < this.shapeCount; i++)
        if (this.shapes[i].CircleCast(ref bodySpaceRay, radius, ref result))
          if (result.IsContained)
            return true;

      // We need to convert the results back to world space to be any use
      // (Doesn't matter if we were contained since there will be no normal)
      if (result.Body == this)
        result.normal = record.BodyToWorldDirection(result.normal);
      return result.IsValid;
    }
    #endregion

    public VoltBody()
    {
      this.Reset();
      this.ProxyId = -1;
    }

    internal void InitializeDynamic(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.Initialize(position, radians, shapesToAdd);
      this.OnPositionUpdated();
      this.ComputeDynamics();
    }

    internal void InitializeStatic(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.Initialize(position, radians, shapesToAdd);
      this.OnPositionUpdated();
      this.SetStatic();
    }

    private void Initialize(
      VoltVector2 position,
      Fix64 radians,
      VoltShape[] shapesToAdd)
    {
      this.Position = position;
      this.Angle = radians;
      this.Facing = VoltMath.Polar(radians);

#if DEBUG
      for (int i = 0; i < shapesToAdd.Length; i++)
        VoltDebug.Assert(shapesToAdd[i].IsInitialized);
#endif

      if ((this.shapes == null) || (this.shapes.Length < shapesToAdd.Length))
        this.shapes = new VoltShape[shapesToAdd.Length];
      Array.Copy(shapesToAdd, this.shapes, shapesToAdd.Length);
      this.shapeCount = shapesToAdd.Length;
      for (int i = 0; i < this.shapeCount; i++)
        this.shapes[i].AssignBody(this);

#if DEBUG
      this.IsInitialized = true;
#endif
    }

    internal void Update()
    {
      if (this.history != null)
        this.history.Store(this.currentState);
      this.Integrate();
      this.OnPositionUpdated();
    }

    internal void AssignWorld(VoltWorld world)
    {
      this.World = world;
    }

    internal void FreeHistory()
    {
      if ((this.World != null) && (this.history != null))
        this.World.FreeHistory(this.history);
      this.history = null;
    }

    internal void FreeShapes()
    {
      if (this.World != null)
      {
        for (int i = 0; i < this.shapeCount; i++)
          this.World.FreeShape(this.shapes[i]);
        for (int i = 0; i < this.shapes.Length; i++)
          this.shapes[i] = null;
      }
      this.shapeCount = 0;
    }

    /// <summary>
    /// Used for saving the body as part of another structure. The body
    /// will retain all geometry data and associated metrics, but its
    /// position, velocity, forces, and all related history will be cleared.
    /// </summary>
    internal void PartialReset()
    {
      this.history = null;
      this.currentState = default(HistoryRecord);

      this.LinearVelocity = VoltVector2.zero;
      this.AngularVelocity = Fix64.Zero;

      this.Force = VoltVector2.zero;
      this.Torque = Fix64.Zero;

      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;
    }

    /// <summary>
    /// Full reset. Clears out all data for pooling. Call FreeShapes() first.
    /// </summary>
    private void Reset()
    {
      VoltDebug.Assert(this.shapeCount == 0);

#if DEBUG
      this.IsInitialized = false;
#endif

      this.UserData = null;
      this.World = null;
      this.BodyType = VoltBodyType.Invalid;
      this.CollisionFilter = null;

      this.Angle = Fix64.Zero;
      this.LinearVelocity = VoltVector2.zero;
      this.AngularVelocity = Fix64.Zero;

      this.Force = VoltVector2.zero;
      this.Torque = Fix64.Zero;

      this.Mass = Fix64.Zero;
      this.Inertia = Fix64.Zero;
      this.InvMass = Fix64.Zero;
      this.InvInertia = Fix64.Zero;

      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;

      this.history = null;
      this.currentState = default(HistoryRecord);
    }

    #region Collision
    internal bool CanCollide(VoltBody other)
    {
      // Ignore self and static-static collisions
      if ((this == other) || (this.IsStatic && other.IsStatic))
        return false;

      if (this.CollisionFilter != null)
        return this.CollisionFilter.Invoke(this, other);
      return true;
    }

    internal void ApplyImpulse(VoltVector2 j, VoltVector2 r)
    {
      this.LinearVelocity += j * this.InvMass;
      this.AngularVelocity -= this.InvInertia * VoltMath.Cross(j, r);
    }

    internal void ApplyBias(VoltVector2 j, VoltVector2 r)
    {
      this.BiasVelocity += j * this.InvMass;
      this.BiasRotation -= this.InvInertia * VoltMath.Cross(j, r);
    }
    #endregion

    #region Transformation Shortcuts
    internal VoltVector2 WorldToBodyPointCurrent(VoltVector2 vector)
    {
      return this.currentState.WorldToBodyPoint(vector);
    }

    internal VoltVector2 BodyToWorldPointCurrent(VoltVector2 vector)
    {
      return this.currentState.BodyToWorldPoint(vector);
    }

    internal Axis BodyToWorldAxisCurrent(Axis axis)
    {
      return this.currentState.BodyToWorldAxis(axis);
    }
    #endregion

    #region Helpers
    /// <summary>
    /// Applies the current position and angle to shapes and the AABB.
    /// </summary>
    private void OnPositionUpdated()
    {
      for (int i = 0; i < this.shapeCount; i++)
        this.shapes[i].OnBodyPositionUpdated();
      this.UpdateAABB();
    }

    /// <summary>
    /// Builds the AABB by combining all the shape AABBs.
    /// </summary>
    private void UpdateAABB()
    {
      Fix64 top = Fix64.MinValue;
      Fix64 right = Fix64.MinValue;
      Fix64 bottom = Fix64.MaxValue;
      Fix64 left = Fix64.MaxValue;

      for (int i = 0; i < this.shapeCount; i++)
      {
        VoltAABB aabb = this.shapes[i].AABB;

        top = VoltMath.Max(top, aabb.Top);
        right = VoltMath.Max(right, aabb.Right);
        bottom = VoltMath.Min(bottom, aabb.Bottom);
        left = VoltMath.Min(left, aabb.Left);
      }

      this.AABB = new VoltAABB(top, bottom, left, right);
    }

    /// <summary>
    /// Computes forces and dynamics and applies them to position and angle.
    /// </summary>
    private void Integrate()
    {
      // Apply damping
      this.LinearVelocity *= this.World.Damping;
      this.AngularVelocity *= this.World.Damping;

      // Calculate total force and torque
      VoltVector2 totalForce = this.Force * this.InvMass;
      Fix64 totalTorque = this.Torque * this.InvInertia;

      // See http://www.niksula.hut.fi/~hkankaan/Homepages/gravity.html
      this.IntegrateForces(totalForce, totalTorque, Fix64.One / (Fix64)2);
      this.IntegrateVelocity();
      this.IntegrateForces(totalForce, totalTorque, Fix64.One / (Fix64)2);

      this.ClearForces();
    }

    private void IntegrateForces(
      VoltVector2 force,
      Fix64 torque,
      Fix64 mult)
    {
      this.LinearVelocity += this.World.DeltaTime * force * mult;
      this.AngularVelocity -= this.World.DeltaTime * torque * mult;
    }

    private void IntegrateVelocity()
    {
      this.Position +=
        this.World.DeltaTime * this.LinearVelocity + this.BiasVelocity;
      this.Angle +=
        this.World.DeltaTime * this.AngularVelocity + this.BiasRotation;
      this.Facing = VoltMath.Polar(this.Angle);
    }

    private void ClearForces()
    {
      this.Force = VoltVector2.zero;
      this.Torque = Fix64.Zero;
      this.BiasVelocity = VoltVector2.zero;
      this.BiasRotation = Fix64.Zero;
    }

    private void ComputeDynamics()
    {
      this.Mass = Fix64.Zero;
      this.Inertia = Fix64.Zero;

      for (int i = 0; i < this.shapeCount; i++)
      {
        VoltShape shape = this.shapes[i];
        if (shape.Density == Fix64.Zero)
          continue;
        Fix64 curMass = shape.Mass;
        Fix64 curInertia = shape.Inertia;

        this.Mass += curMass;
        this.Inertia += curMass * curInertia;
      }

      if (this.Mass < VoltConfig.MINIMUM_DYNAMIC_MASS)
      {
        throw new InvalidOperationException("Mass of dynamic too small");
      }
      else
      {
        this.InvMass = Fix64.One / this.Mass;
        this.InvInertia = Fix64.One / this.Inertia;
      }

      this.BodyType = VoltBodyType.Dynamic;
    }

    private void SetStatic()
    {
      this.Mass = Fix64.Zero;
      this.Inertia = Fix64.Zero;
      this.InvMass = Fix64.Zero;
      this.InvInertia = Fix64.Zero;

      this.BodyType = VoltBodyType.Static;
    }
#endregion

    #region Debug
    public void GizmoDraw(
      IGizmosDrawer drawer,
      Color edgeColor,
      Color normalColor,
      Color bodyOriginColor,
      Color shapeOriginColor,
      Color bodyAabbColor,
      Color shapeAabbColor,
      Fix64 normalLength)
    {
      // Draw origin
      drawer.DrawCircle(this.Position, 0.1f, bodyOriginColor);

      // Draw facing
      drawer.DrawLine(
        this.Position,
        this.Position + this.Facing * normalLength, normalColor);

      this.AABB.GizmoDraw(drawer, bodyAabbColor);

      for (int i = 0; i < this.shapeCount; i++)
        this.shapes[i].GizmoDraw(
          drawer,
          edgeColor,
          normalColor,
          shapeOriginColor,
          shapeAabbColor,
          normalLength);
    }

    public void GizmoDrawHistory(IGizmosDrawer drawer, Color aabbColor)
    {
      if (this.history != null)
        foreach (HistoryRecord record in this.history.GetValues())
          record.aabb.GizmoDraw(drawer, aabbColor);
    }
    #endregion
  }
}