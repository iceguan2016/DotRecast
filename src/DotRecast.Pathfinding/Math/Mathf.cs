// Unity C# reference source
// Copyright (c) Unity Technologies. For terms of use, see
// https://unity3d.com/legal/licenses/Unity_Reference_Only_License

using System;
using System.Threading;
using System.Runtime.CompilerServices;

namespace UnityEngineInternal
{
    public partial struct MathfInternal
    {
        public static volatile float FloatMinNormal = 1.17549435E-38f;
        public static volatile float FloatMinDenormal = Single.Epsilon;

        public static bool IsFlushToZeroEnabled = (FloatMinDenormal == 0);
    }
}

namespace UnityEngine
{
    // A collection of common math functions.
    public partial struct Mathf
    {
        // Returns the sine of angle /f/ in radians.
        public static float Sin(float f) { return (float)Math.Sin(f); }

        // Returns the cosine of angle /f/ in radians.
        public static float Cos(float f) { return (float)Math.Cos(f); }

        // Returns the tangent of angle /f/ in radians.
        public static float Tan(float f) { return (float)Math.Tan(f); }

        // Returns the arc-sine of /f/ - the angle in radians whose sine is /f/.
        public static float Asin(float f) { return (float)Math.Asin(f); }

        // Returns the arc-cosine of /f/ - the angle in radians whose cosine is /f/.
        public static float Acos(float f) { return (float)Math.Acos(f); }

        // Returns the arc-tangent of /f/ - the angle in radians whose tangent is /f/.
        public static float Atan(float f) { return (float)Math.Atan(f); }

        // Returns the angle in radians whose ::ref::Tan is @@y/x@@.
        public static float Atan2(float y, float x) { return (float)Math.Atan2(y, x); }

        // Returns square root of /f/.
        public static float Sqrt(float f) { return (float)Math.Sqrt(f); }

        // Returns the absolute value of /f/.
        public static float Abs(float f) { return Math.Abs(f); }

        // Returns the absolute value of /value/.
        public static int Abs(int value) { return Math.Abs(value); }

        /// *listonly*
        public static float Min(float a, float b) { return a < b ? a : b; }
        // Returns the smallest of two or more values.
        public static float Min(params float[] values)
        {
            int len = values.Length;
            if (len == 0)
                return 0;
            float m = values[0];
            for (int i = 1; i < len; i++)
            {
                if (values[i] < m)
                    m = values[i];
            }
            return m;
        }

        /// *listonly*
        public static int Min(int a, int b) { return a < b ? a : b; }
        // Returns the smallest of two or more values.
        public static int Min(params int[] values)
        {
            int len = values.Length;
            if (len == 0)
                return 0;
            int m = values[0];
            for (int i = 1; i < len; i++)
            {
                if (values[i] < m)
                    m = values[i];
            }
            return m;
        }

        /// *listonly*
        public static float Max(float a, float b) { return a > b ? a : b; }
        // Returns largest of two or more values.
        public static float Max(params float[] values)
        {
            int len = values.Length;
            if (len == 0)
                return 0;
            float m = values[0];
            for (int i = 1; i < len; i++)
            {
                if (values[i] > m)
                    m = values[i];
            }
            return m;
        }

        /// *listonly*
        public static int Max(int a, int b) { return a > b ? a : b; }
        // Returns the largest of two or more values.
        public static int Max(params int[] values)
        {
            int len = values.Length;
            if (len == 0)
                return 0;
            int m = values[0];
            for (int i = 1; i < len; i++)
            {
                if (values[i] > m)
                    m = values[i];
            }
            return m;
        }

        // Returns /f/ raised to power /p/.
        public static float Pow(float f, float p) { return (float)Math.Pow(f, p); }

        // Returns e raised to the specified power.
        public static float Exp(float power) { return (float)Math.Exp(power); }

        // Returns the logarithm of a specified number in a specified base.
        public static float Log(float f, float p) { return (float)Math.Log(f, p); }

        // Returns the natural (base e) logarithm of a specified number.
        public static float Log(float f) { return (float)Math.Log(f); }

        // Returns the base 10 logarithm of a specified number.
        public static float Log10(float f) { return (float)Math.Log10(f); }

        // Returns the smallest integer greater to or equal to /f/.
        public static float Ceil(float f) { return (float)Math.Ceiling(f); }

        // Returns the largest integer smaller to or equal to /f/.
        public static float Floor(float f) { return (float)Math.Floor(f); }

        // Returns /f/ rounded to the nearest integer.
        public static float Round(float f) { return (float)Math.Round(f); }

        // Returns the smallest integer greater to or equal to /f/.
        public static int CeilToInt(float f) { return (int)Math.Ceiling(f); }

        // Returns the largest integer smaller to or equal to /f/.
        public static int FloorToInt(float f) { return (int)Math.Floor(f); }

        // Returns /f/ rounded to the nearest integer.
        public static int RoundToInt(float f) { return (int)Math.Round(f); }

        // Returns the sign of /f/.
        [MethodImpl(MethodImplOptionsEx.AggressiveInlining)]
        public static float Sign(float f) { return f >= 0F ? 1F : -1F; }

        // The infamous ''3.14159265358979...'' value (RO).
        public const float PI = (float)Math.PI;

        // A representation of positive infinity (RO).
        public const float Infinity = Single.PositiveInfinity;

        // A representation of negative infinity (RO).
        public const float NegativeInfinity = Single.NegativeInfinity;

        // Degrees-to-radians conversion constant (RO).
        public const float Deg2Rad = PI * 2F / 360F;

        // Radians-to-degrees conversion constant (RO).
        public const float Rad2Deg = 1F / Deg2Rad;

        // We cannot round to more decimals than 15 according to docs for System.Math.Round.
        internal const int kMaxDecimals = 15;

        // A tiny floating point value (RO).
        public static readonly float Epsilon =
            UnityEngineInternal.MathfInternal.IsFlushToZeroEnabled ? UnityEngineInternal.MathfInternal.FloatMinNormal
            : UnityEngineInternal.MathfInternal.FloatMinDenormal;

        // Clamps a value between a minimum float and maximum float value.
        public static float Clamp(float value, float min, float max)
        {
            if (value < min)
                value = min;
            else if (value > max)
                value = max;
            return value;
        }

        // Clamps value between min and max and returns value.
        // Set the position of the transform to be that of the time
        // but never less than 1 or more than 3
        //
        public static int Clamp(int value, int min, int max)
        {
            if (value < min)
                value = min;
            else if (value > max)
                value = max;
            return value;
        }

        // Clamps value between 0 and 1 and returns value
        public static float Clamp01(float value)
        {
            if (value < 0F)
                return 0F;
            else if (value > 1F)
                return 1F;
            else
                return value;
        }

        // Interpolates between /a/ and /b/ by /t/. /t/ is clamped between 0 and 1.
        public static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * Clamp01(t);
        }

        // Interpolates between /a/ and /b/ by /t/ without clamping the interpolant.
        public static float LerpUnclamped(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        // Same as ::ref::Lerp but makes sure the values interpolate correctly when they wrap around 360 degrees.
        public static float LerpAngle(float a, float b, float t)
        {
            float delta = Repeat((b - a), 360);
            if (delta > 180)
                delta -= 360;
            return a + delta * Clamp01(t);
        }

        // Moves a value /current/ towards /target/.
        static public float MoveTowards(float current, float target, float maxDelta)
        {
            if (Mathf.Abs(target - current) <= maxDelta)
                return target;
            return current + Mathf.Sign(target - current) * maxDelta;
        }

        // Same as ::ref::MoveTowards but makes sure the values interpolate correctly when they wrap around 360 degrees.
        static public float MoveTowardsAngle(float current, float target, float maxDelta)
        {
            float deltaAngle = DeltaAngle(current, target);
            if (-maxDelta < deltaAngle && deltaAngle < maxDelta)
                return target;
            target = current + deltaAngle;
            return MoveTowards(current, target, maxDelta);
        }

        // Interpolates between /min/ and /max/ with smoothing at the limits.
        public static float SmoothStep(float from, float to, float t)
        {
            t = Mathf.Clamp01(t);
            t = -2.0F * t * t * t + 3.0F * t * t;
            return to * t + from * (1F - t);
        }

        //*undocumented
        public static float Gamma(float value, float absmax, float gamma)
        {
            bool negative = value < 0F;
            float absval = Abs(value);
            if (absval > absmax)
                return negative ? -absval : absval;

            float result = Pow(absval / absmax, gamma) * absmax;
            return negative ? -result : result;
        }

        // Compares two floating point values if they are similar.
        public static bool Approximately(float a, float b)
        {
            // If a or b is zero, compare that the other is less or equal to epsilon.
            // If neither a or b are 0, then find an epsilon that is good for
            // comparing numbers at the maximum magnitude of a and b.
            // Floating points have about 7 significant digits, so
            // 1.000001f can be represented while 1.0000001f is rounded to zero,
            // thus we could use an epsilon of 0.000001f for comparing values close to 1.
            // We multiply this epsilon by the biggest magnitude of a and b.
            return Abs(b - a) < Max(0.000001f * Max(Abs(a), Abs(b)), Epsilon * 8);
        }

        // Loops the value t, so that it is never larger than length and never smaller than 0.
        public static float Repeat(float t, float length)
        {
            return Clamp(t - Mathf.Floor(t / length) * length, 0.0f, length);
        }

        // PingPongs the value t, so that it is never larger than length and never smaller than 0.
        public static float PingPong(float t, float length)
        {
            t = Repeat(t, length * 2F);
            return length - Mathf.Abs(t - length);
        }

        // Calculates the ::ref::Lerp parameter between of two values.
        public static float InverseLerp(float a, float b, float value)
        {
            if (a != b)
                return Clamp01((value - a) / (b - a));
            else
                return 0.0f;
        }

        // Calculates the shortest difference between two given angles.
        public static float DeltaAngle(float current, float target)
        {
            float delta = Mathf.Repeat((target - current), 360.0F);
            if (delta > 180.0F)
                delta -= 360.0F;
            return delta;
        }

        // Infinite Line Intersection (line1 is p1-p2 and line2 is p3-p4)
        internal static bool LineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 result)
        {
            float bx = p2.x - p1.x;
            float by = p2.y - p1.y;
            float dx = p4.x - p3.x;
            float dy = p4.y - p3.y;
            float bDotDPerp = bx * dy - by * dx;
            if (bDotDPerp == 0)
            {
                return false;
            }
            float cx = p3.x - p1.x;
            float cy = p3.y - p1.y;
            float t = (cx * dy - cy * dx) / bDotDPerp;

            result.x = p1.x + t * bx;
            result.y = p1.y + t * by;
            return true;
        }

        // Line Segment Intersection (line1 is p1-p2 and line2 is p3-p4)
        internal static bool LineSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 result)
        {
            float bx = p2.x - p1.x;
            float by = p2.y - p1.y;
            float dx = p4.x - p3.x;
            float dy = p4.y - p3.y;
            float bDotDPerp = bx * dy - by * dx;
            if (bDotDPerp == 0)
            {
                return false;
            }
            float cx = p3.x - p1.x;
            float cy = p3.y - p1.y;
            float t = (cx * dy - cy * dx) / bDotDPerp;
            if (t < 0 || t > 1)
            {
                return false;
            }
            float u = (cx * by - cy * bx) / bDotDPerp;
            if (u < 0 || u > 1)
            {
                return false;
            }

            result.x = p1.x + t * bx;
            result.y = p1.y + t * by;
            return true;
        }

        static internal long RandomToLong(System.Random r)
        {
            var buffer = new byte[8];
            r.NextBytes(buffer);
            return (long)(System.BitConverter.ToUInt64(buffer, 0) & System.Int64.MaxValue);
        }

        internal static float ClampToFloat(double value)
        {
            if (double.IsPositiveInfinity(value))
                return float.PositiveInfinity;

            if (double.IsNegativeInfinity(value))
                return float.NegativeInfinity;

            if (value < float.MinValue)
                return float.MinValue;

            if (value > float.MaxValue)
                return float.MaxValue;

            return (float)value;
        }

        internal static int ClampToInt(long value)
        {
            if (value < int.MinValue)
                return int.MinValue;

            if (value > int.MaxValue)
                return int.MaxValue;

            return (int)value;
        }

        internal static uint ClampToUInt(long value)
        {
            if (value < uint.MinValue)
                return uint.MinValue;

            if (value > uint.MaxValue)
                return uint.MaxValue;

            return (uint)value;
        }

        internal static float RoundToMultipleOf(float value, float roundingValue)
        {
            if (roundingValue == 0)
                return value;
            return Mathf.Round(value / roundingValue) * roundingValue;
        }

        internal static float GetClosestPowerOfTen(float positiveNumber)
        {
            if (positiveNumber <= 0)
                return 1;
            return Mathf.Pow(10, Mathf.RoundToInt(Mathf.Log10(positiveNumber)));
        }

        internal static int GetNumberOfDecimalsForMinimumDifference(float minDifference)
        {
            return Mathf.Clamp(-Mathf.FloorToInt(Mathf.Log10(Mathf.Abs(minDifference))), 0, kMaxDecimals);
        }

        internal static int GetNumberOfDecimalsForMinimumDifference(double minDifference)
        {
            return (int)System.Math.Max(0.0, -System.Math.Floor(System.Math.Log10(System.Math.Abs(minDifference))));
        }

        internal static float RoundBasedOnMinimumDifference(float valueToRound, float minDifference)
        {
            if (minDifference == 0)
                return DiscardLeastSignificantDecimal(valueToRound);
            return (float)System.Math.Round(valueToRound, GetNumberOfDecimalsForMinimumDifference(minDifference),
                System.MidpointRounding.AwayFromZero);
        }

        internal static double RoundBasedOnMinimumDifference(double valueToRound, double minDifference)
        {
            if (minDifference == 0)
                return DiscardLeastSignificantDecimal(valueToRound);
            return System.Math.Round(valueToRound, GetNumberOfDecimalsForMinimumDifference(minDifference),
                System.MidpointRounding.AwayFromZero);
        }

        internal static float DiscardLeastSignificantDecimal(float v)
        {
            int decimals = Mathf.Clamp((int)(5 - Mathf.Log10(Mathf.Abs(v))), 0, kMaxDecimals);
            return (float)System.Math.Round(v, decimals, System.MidpointRounding.AwayFromZero);
        }

        internal static double DiscardLeastSignificantDecimal(double v)
        {
            int decimals = System.Math.Max(0, (int)(5 - System.Math.Log10(System.Math.Abs(v))));
            try
            {
                return System.Math.Round(v, decimals);
            }
            catch (System.ArgumentOutOfRangeException)
            {
                // This can happen for very small numbers.
                return 0;
            }
        }

        public static int NextPowerOfTwo(int value)
        {
            value -= 1;
            value |= value >> 16;
            value |= value >> 8;
            value |= value >> 4;
            value |= value >> 2;
            value |= value >> 1;
            return value + 1;
        }
        public static int ClosestPowerOfTwo(int value)
        {
            int nextPower = NextPowerOfTwo(value);
            int prevPower = nextPower >> 1;
            if (value - prevPower < nextPower - value)

                return prevPower;
            else
                return nextPower;
        }
        public static bool IsPowerOfTwo(int value)
        {
            return (value & (value - 1)) == 0;
        }
    }
}
