// Copyright (c) 2002-2003, Sony Computer Entertainment America
// Copyright (c) 2002-2003, Craig Reynolds <craig_reynolds@playstation.sony.com>
// Copyright (C) 2007 Bjoern Graf <bjoern.graf@gmx.net>
// Copyright (C) 2007 Michael Coles <michael@digini.com>
// All rights reserved.
//
// This software is licensed as described in the file license.txt, which
// you should have received as part of this distribution. The terms
// are also available at http://www.codeplex.com/SharpSteer/Project/License.aspx.

using System;
using System.Collections.Generic;
using System.Numerics;

namespace SharpSteer2.Helpers
{
	public class Utilities
	{
		public static FixMath.F64 square(FixMath.F64 x) { return x * x; }
        public static FixMath.F64 floorXXX(FixMath.F64 x) { return FixMath.F64.Floor(x); }
        public static FixMath.F64 sqrtXXX(FixMath.F64 x) { return FixMath.F64.Sqrt(x); }
        public static FixMath.F64 sinXXX(FixMath.F64 x) { return FixMath.F64.Sin(x); }
        public static FixMath.F64 cosXXX(FixMath.F64 x) { return FixMath.F64.Cos(x); }
        public static FixMath.F64 absXXX(FixMath.F64 x) { return FixMath.F64.Abs(x); }
        public static int absXXX(int x) { return Math.Abs(x); }
        public static FixMath.F64 maxXXX(FixMath.F64 x, FixMath.F64 y) { if (x > y) return x; else return y; }
        public static FixMath.F64 minXXX(FixMath.F64 x, FixMath.F64 y) { if (x < y) return x; else return y; }

        /// <summary>
        /// Linearly interpolate from A to B by amount T
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
	    public static FixMath.F64 Lerp(FixMath.F64 a, FixMath.F64 b, FixMath.F64 t)
        {
            return a + (b - a) * t;
        }

        /// <summary>
        /// Clamp value between min and max
        /// </summary>
        /// <param name="value"></param>
        /// <param name="min"></param>
        /// <param name="max"></param>
        /// <returns></returns>
	    public static FixMath.F64 Clamp(FixMath.F64 value, FixMath.F64 min, FixMath.F64 max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
        }

	    /// <summary>
        /// remap a value specified relative to a pair of bounding values to the corresponding value relative to another pair of bounds.
        /// </summary>
        /// <remarks>Inspired by (dyna:remap-interval y y0 y1 z0 z1)</remarks>
        /// <param name="x">A value</param>
        /// <param name="in0">Starting lower bound</param>
        /// <param name="in1">Starting upper bound</param>
        /// <param name="out0">Ending lower bound</param>
        /// <param name="out1">Ending upper bound</param>
        /// <returns></returns>
		public static FixMath.F64 RemapInterval(FixMath.F64 x, FixMath.F64 in0, FixMath.F64 in1, FixMath.F64 out0, FixMath.F64 out1)
		{
			// uninterpolate: what is x relative to the interval in0:in1?
			var relative = (x - in0) / (in1 - in0);

			// now interpolate between output interval based on relative x
			return Lerp(out0, out1, relative);
		}

        /// <summary>
        /// Like remapInterval but the result is clipped to remain between out0 and out1
        /// </summary>
        /// <param name="x">A value</param>
        /// <param name="in0">Starting lower bound</param>
        /// <param name="in1">Starting upper bound</param>
        /// <param name="out0">Ending lower bound</param>
        /// <param name="out1">Ending upper bound</param>
        /// <returns></returns>
		public static FixMath.F64 RemapIntervalClip(FixMath.F64 x, FixMath.F64 in0, FixMath.F64 in1, FixMath.F64 out0, FixMath.F64 out1)
		{
			// uninterpolate: what is x relative to the interval in0:in1?
			var relative = (x - in0) / (in1 - in0);

			// now interpolate between output interval based on relative x
            return Lerp(out0, out1, Clamp(relative, FixMath.F64.Zero, FixMath.F64.One));
		}

        /// <summary>
        /// classify a value relative to the interval between two bounds:
        /// returns -1 when below the lower bound, returns  0 when between the bounds (inside the interval), returns +1 when above the upper bound
        /// </summary>
        /// <param name="x"></param>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <returns></returns>
		public static int IntervalComparison(FixMath.F64 x, FixMath.F64 lowerBound, FixMath.F64 upperBound)
		{
			if (x < lowerBound) return -1;
			if (x > upperBound) return +1;
			return 0;
		}

		public static FixMath.F64 ScalarRandomWalk(FixMath.F64 initial, FixMath.F64 walkspeed, FixMath.F64 min, FixMath.F64 max)
		{
			var next = initial + (((RandomHelpers.Random() * 2) - 1) * walkspeed);
			if (next < min) return min;
			if (next > max) return max;
			return next;
		}

		/// <summary>
		/// blends new values into an accumulator to produce a smoothed time series
		/// </summary>
		/// <remarks>
		/// Modifies its third argument, a reference to the float accumulator holding
		/// the "smoothed time series."
		/// 
		/// The first argument (smoothRate) is typically made proportional to "dt" the
		/// simulation time step.  If smoothRate is 0 the accumulator will not change,
		/// if smoothRate is 1 the accumulator will be set to the new value with no
		/// smoothing.  Useful values are "near zero".
		/// </remarks>
		/// <param name="smoothRate"></param>
		/// <param name="newValue"></param>
		/// <param name="smoothedAccumulator"></param>
		/// <example>blendIntoAccumulator (dt * 0.4f, currentFPS, smoothedFPS)</example>
		public static void BlendIntoAccumulator(FixMath.F64 smoothRate, FixMath.F64 newValue, ref FixMath.F64 smoothedAccumulator)
		{
            smoothedAccumulator = Lerp(smoothedAccumulator, newValue, Clamp(smoothRate, FixMath.F64.Zero, FixMath.F64.One));
		}

		public static void BlendIntoAccumulator(FixMath.F64 smoothRate, FixMath.F64Vec3 newValue, ref FixMath.F64Vec3 smoothedAccumulator)
		{
            smoothedAccumulator = FixMath.F64Vec3.Lerp(smoothedAccumulator, newValue, Clamp(smoothRate, FixMath.F64.Zero, FixMath.F64.One));
		}

        public static void Swap<T>(List<T> list, int i, int j)
        {
            T temp = list[i];
            list[i] = list[j];
            list[j] = temp;
        }

        public static void RemoveAtSwap<T>(List<T> list, int i)
        {
            var lastIndex = list.Count - 1;
            Swap(list, i, lastIndex);
            list.RemoveAt(lastIndex);
        }

        public static FixMath.F64 GetRangePct(FixMath.F64Vec2 Range, FixMath.F64 Value)
        {
	        return (Range.X != Range.Y) ? (Value - Range.X) / (Range.Y - Range.X) : Range.X;
        }

        public static FixMath.F64 GetRangeValue(FixMath.F64Vec2 Range, FixMath.F64 Pct)
        {
	        return FixMath.F64.Lerp(Range.X, Range.Y, Pct);
        }

        /** For the given Value clamped to the [Input:Range] inclusive, returns the corresponding percentage in [Output:Range] Inclusive. */
        public static FixMath.F64 GetMappedRangeValueClamped(FixMath.F64Vec2 InputRange, FixMath.F64Vec2 OutputRange, FixMath.F64 Value)
	    {

            var ClampedPct = FixMath.F64.Clamp01(GetRangePct(InputRange, Value));
		    return GetRangeValue(OutputRange, ClampedPct);
        }

        /** Transform the given Value relative to the input range to the Output Range. */
        public static FixMath.F64 GetMappedRangeValueUnclamped(FixMath.F64Vec2 InputRange, FixMath.F64Vec2 OutputRange, FixMath.F64 Value)
        {
            return GetRangeValue(OutputRange, GetRangePct(InputRange, Value));
        }
    }
}
