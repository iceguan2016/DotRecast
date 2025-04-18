using System;
using Pathfinding.Util;

namespace DotRecast.Pathfinding.Util
{
    public static class Reduction
    {
        /// <summary>
        /// Counts the leading zeros in a 32-bit integer.
        /// </summary>
        /// <param name="value">The 32-bit unsigned integer.</param>
        /// <returns>The number of leading zero bits.</returns>
        public static int LeadingZeroCount(uint value)
        {
            if (value == 0)
                return 32;

            int count = 0;
            while ((value & 0x80000000U) == 0)
            {
                count++;
                value <<= 1;
            }
            return count;
        }

        /// <summary>
        /// Counts the leading zeros in a 64-bit integer.
        /// </summary>
        /// <param name="value">The 64-bit unsigned integer.</param>
        /// <returns>The number of leading zero bits.</returns>
        public static int LeadingZeroCount(ulong value)
        {
            if (value == 0)
                return 64;

            int count = 0;
            while ((value & 0x8000000000000000UL) == 0)
            {
                count++;
                value <<= 1;
            }
            return count;
        }

        // reduce into[-2^A, 2^A), 返回shift，表示需要缩放1<<shift倍
        // BitOperations.LeadingZeroCount(value): 返回 value 的二进制表示中前导零的个数
        //public static int reduction_shift(this FixMath.F32 self, in int A)
        //{
        //    // assume leading zeros is l
        //    // then self is in [0, 2^(32 - l - F))
        //    // then shift = A - (32 - l - F)
        //    var bit = 32;
        //    var rawAbs = Math.Abs(self.Raw);
        //    var leadingZeros = LeadingZeroCount((uint)rawAbs);
        //    return A - (bit - leadingZeros - FixPointCS.Fixed32.Shift);
        //}

        public static int reduction_shift(ref this FixMath.F64 self, in int A)
        {
            // assume leading zeros is l
            // then self is in [0, 2^(64 - l - F))
            // then shift = A - (64 - l - F)
            var bit = 64;
            var rawAbs = Math.Abs(self.Raw);
            var leadingZeros = LeadingZeroCount((ulong)rawAbs);
            return A - (bit - leadingZeros - FixPointCS.Fixed64.Shift);
        }

        public static FixMath.F64 reduce_into(this FixMath.F64 self, in int shift) 
        {
            var R = FixPointCS.Fixed64.Shift;
            var F = FixPointCS.Fixed64.Shift;
            var shift2 = R - F + shift;
            if (shift2 < 0) 
                return FixMath.F64.FromRaw(self.Raw >> (-shift2));
            else if (shift2 == 0) 
                return FixMath.F64.FromRaw(self.Raw);
            else 
                return FixMath.F64.FromRaw(self.Raw << shift2);
        }

        public static FixMath.F64 reproduce_from(FixMath.F64 reducted, in int shift)
        {
            return reducted.reduce_into(-shift);
        }

        public static void test_reduce_64()
        {
            // reduce into [-2, 2)
            const int A = 1;

            // normal case
            {
                var x = FixMath.F64.FromDouble(0.01171875);
                Debug.Assert(0.01171875 * (1 << 7) == 1.5);

                var shift = x.reduction_shift(A);
                Debug.Assert(shift == 7);
                var reducted = FixMath.F64.FromDouble(1.5);
                var a = x.reduce_into(shift);
                Debug.Assert(a == reducted); // 缩放到 [-2, 2)
                var b = reproduce_from(reducted, shift);
                Debug.Assert(b == x); // 还原会原始值0.01171875
            }

            // edge case
            {
                var x = FixMath.F64.FromDouble(-0.125);
                Debug.Assert(-0.125 * (1 << 3) == -1);
                var shift = x.reduction_shift(A);
                Debug.Assert(shift == 3);
                var reducted = FixMath.F64.FromDouble(-1);
                var a = x.reduce_into(shift);
                Debug.Assert(a == reducted); // 缩放到 [-2, 2)
                var b = reproduce_from(reducted, shift);
                Debug.Assert(b == x); // 还原会原始值-0.125
            }

            // zero case
            {
                var x = FixMath.F64.FromDouble(0.0);
                var shift = x.reduction_shift(A);
                var a = x.reduce_into(shift);
                Debug.Assert(a == FixMath.F64.Zero); // 缩放到 [-2, 2)
                var b = reproduce_from(FixMath.F64.Zero, shift);
                Debug.Assert(b == x); // 还原会原始值
            }
        }

        // FixMath.F64Vec2
        public static FixMath.F64 abs_max_element(this FixMath.F64Vec2 v)
        { 
            return FixMath.F64.Max(FixMath.F64.Abs(v.X) ,FixMath.F64.Abs(v.Y));
        }

        public static FixMath.F64Vec2 reduce_into(this FixMath.F64Vec2 v, in int shift)
        {
            return new FixMath.F64Vec2(v.X.reduce_into(shift), v.Y.reduce_into(shift));
        }


    }
}
