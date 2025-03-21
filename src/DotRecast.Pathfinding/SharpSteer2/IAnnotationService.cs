// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using SharpSteer2.Obstacles;

namespace SharpSteer2
{
	/// <summary>
	/// Provides methods to annotate the steering behaviors.
	/// </summary>
	public interface IAnnotationService
	{
		/// <summary>
		/// Indicates whether annotation is enabled.
		/// </summary>
		bool IsEnabled { get; set; }

		// ------------------------------------------------------------------------
		// drawing of lines, circles and (filled) disks to annotate steering
		// behaviors.  When called during OpenSteerDemo's simulation update phase,
		// these functions call a "deferred draw" routine which buffer the
		// arguments for use during the redraw phase.
		//
		// note: "circle" means unfilled
		//       "disk" means filled
		//       "XZ" means on a plane parallel to the X and Z axes (perp to Y)
		//       "3d" means the circle is perpendicular to the given "axis"
		//       "segments" is the number of line segments used to draw the circle

	    /// <summary>
	    /// Draws an opaque colored line segment between two locations in space.
	    /// </summary>
	    /// <param name="startPoint">The start point of the line.</param>
	    /// <param name="endPoint">The end point of the line.</param>
	    /// <param name="color">The color of the line.</param>
	    /// <param name="opacity"></param>
	    void Line(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec3 color, FixMath.F64 opacity /*= 1.0f*/);

        void Arrow(FixMath.F64Vec3 startPoint, FixMath.F64Vec3 endPoint, FixMath.F64Vec2 arrowSize, FixMath.F64 lineWidth, FixMath.F64Vec3 color, FixMath.F64 opacity);

        /// <summary>
        /// Draws a circle on the XZ plane.
        /// </summary>
        /// <param name="radius">The radius of the circle.</param>
        /// <param name="center">The center of the circle.</param>
        /// <param name="color">The color of the circle.</param>
        /// <param name="segments">The number of segments to use to draw the circle.</param>
        void CircleXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments);

		/// <summary>
		/// Draws a disk on the XZ plane.
		/// </summary>
		/// <param name="radius">The radius of the disk.</param>
		/// <param name="center">The center of the disk.</param>
		/// <param name="color">The color of the disk.</param>
		/// <param name="segments">The number of segments to use to draw the disk.</param>
        void DiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments);

		/// <summary>
		/// Draws a circle perpendicular to the given axis.
		/// </summary>
		/// <param name="radius">The radius of the circle.</param>
		/// <param name="center">The center of the circle.</param>
		/// <param name="axis">The axis of the circle.</param>
		/// <param name="color">The color of the circle.</param>
		/// <param name="segments">The number of segments to use to draw the circle.</param>
        void Circle3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments);

		/// <summary>
		/// Draws a disk perpendicular to the given axis.
		/// </summary>
		/// <param name="radius">The radius of the disk.</param>
		/// <param name="center">The center of the disk.</param>
		/// <param name="axis">The axis of the disk.</param>
		/// <param name="color">The color of the disk.</param>
		/// <param name="segments">The number of segments to use to draw the disk.</param>
        void Disk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments);

		/// <summary>
		/// Draws a circle (not filled) or disk (filled) on the XZ plane.
		/// </summary>
		/// <param name="radius">The radius of the circle/disk.</param>
		/// <param name="center">The center of the circle/disk.</param>
		/// <param name="color">The color of the circle/disk.</param>
		/// <param name="segments">The number of segments to use to draw the circle/disk.</param>
		/// <param name="filled">Flag indicating whether to draw a disk or circle.</param>
        void CircleOrDiskXZ(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled);

		/// <summary>
		/// Draws a circle (not filled) or disk (filled) perpendicular to the given axis.
		/// </summary>
		/// <param name="radius">The radius of the circle/disk.</param>
		/// <param name="center">The center of the circle/disk.</param>
		/// <param name="axis">The axis of the circle/disk.</param>
		/// <param name="color">The color of the circle/disk.</param>
		/// <param name="segments">The number of segments to use to draw the circle/disk.</param>
		/// <param name="filled">Flag indicating whether to draw a disk or circle.</param>
        void CircleOrDisk3D(FixMath.F64 radius, FixMath.F64Vec3 center, FixMath.F64Vec3 axis, FixMath.F64Vec3 color, int segments, bool filled);

		/// <summary>
		/// Draws a circle (not filled) or disk (filled) perpendicular to the given axis.
		/// </summary>
		/// <param name="radius">The radius of the circle/disk.</param>
		/// <param name="axis">The axis of the circle/disk.</param>
		/// <param name="center">The center of the circle/disk.</param>
		/// <param name="color">The color of the circle/disk.</param>
		/// <param name="segments">The number of segments to use to draw the circle/disk.</param>
		/// <param name="filled">Flag indicating whether to draw a disk or circle.</param>
		/// <param name="in3D">Flag indicating whether to draw the disk/circle in 3D or the XZ plane.</param>
        void CircleOrDisk(FixMath.F64 radius, FixMath.F64Vec3 axis, FixMath.F64Vec3 center, FixMath.F64Vec3 color, int segments, bool filled, bool in3D);

        void SolidPlane(FixMath.F64Vec3 point, FixMath.F64Vec3 normal, FixMath.F64Vec2 size, FixMath.F64Vec3 color, FixMath.F64 opacity);

        void SolidCube(FixMath.F64Vec3 center, FixMath.F64Vec3 size, FixMath.F64Vec3 color, FixMath.F64 opacity);

        /// <summary>
        /// Called when steerToAvoidObstacles decides steering is required.
        /// </summary>
        /// <param name="minDistanceToCollision"></param>
        void AvoidObstacle(IVehicle vehicle, FixMath.F64 minDistanceToCollision);

        // called when steerToAvoidObstacles decides steering is required
        // (default action is to do nothing, layered classes can overload it)
        void AvoidObstacle(IVehicle vehicle, FixMath.F64 minDistanceToCollision, Obstacles.PathIntersection nearest);

		/// <summary>
		/// Called when steerToFollowPath decides steering is required.
		/// </summary>
		/// <param name="future"></param>
		/// <param name="onPath"></param>
		/// <param name="target"></param>
		/// <param name="outside"></param>
		void PathFollowing(IVehicle vehicle, FixMath.F64Vec3 future, FixMath.F64Vec3 onPath, FixMath.F64Vec3 target, FixMath.F64 outside);

		/// <summary>
		/// Called when steerToAvoidCloseNeighbors decides steering is required.
		/// </summary>
		/// <param name="other"></param>
		/// <param name="additionalDistance"></param>
		void AvoidCloseNeighbor(IVehicle vehicle, IVehicle other, FixMath.F64 additionalDistance);
        void AvoidCloseNeighbor(IVehicle vehicle, IVehicle threat, FixMath.F64Vec3 avoidDirection, IVehicle.FAvoidNeighborInfo info);

        /// <summary>
        /// Called when steerToAvoidNeighbors decides steering is required.
        /// </summary>
        /// <param name="threat"></param>
        /// <param name="steer"></param>
        /// <param name="ourFuture"></param>
        /// <param name="threatFuture"></param>
        void AvoidNeighbor(IVehicle vehicle, IVehicle threat, FixMath.F64 steer, FixMath.F64Vec3 ourFuture, FixMath.F64Vec3 threatFuture);
        void AvoidNeighbor(IVehicle vehicle, IVehicle threat, PathIntersection intersection);

        /// <summary>
        /// Draws lines from the vehicle's position showing its velocity and acceleration.
        /// </summary>
        /// <param name="vehicle">The vehicle to annotate.</param>
        void VelocityAcceleration(IVehicle vehicle);

		/// <summary>
		/// Draws lines from the vehicle's position showing its velocity and acceleration.
		/// </summary>
		/// <param name="vehicle">The vehicle to annotate.</param>
		/// <param name="maxLength">The maximum length for the acceleration and velocity lines.</param>
		void VelocityAcceleration(IVehicle vehicle, FixMath.F64 maxLength);

		/// <summary>
		/// Draws lines from the vehicle's position showing its velocity and acceleration.
		/// </summary>
		/// <param name="vehicle">The vehicle to annotate.</param>
		/// <param name="maxLengthAcceleration">The maximum length for the acceleration line.</param>
		/// <param name="maxLengthVelocity">The maximum length for the velocity line.</param>
		void VelocityAcceleration(IVehicle vehicle, FixMath.F64 maxLengthAcceleration, FixMath.F64 maxLengthVelocity);
	}
}
