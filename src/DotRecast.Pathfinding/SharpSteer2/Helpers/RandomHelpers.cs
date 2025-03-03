using System;

namespace SharpSteer2.Helpers
{
    public class RandomHelpers
    {
        [ThreadStatic]
        private static Random _rng = null;

        private static Random rng
        {
            get
            {
                return _rng;
            }
        }

        public static void InitialRandom(int Seed)
        {
            _rng = new Random(Seed);
        }

        /// <summary>
        /// Returns a float randomly distributed between 0 and 1
        /// </summary>
        /// <returns></returns>
        public static FixMath.F64 Random()
        {
            var F32_01 = FixMath.F32.FromRaw(rng.Next(FixMath.F32.One.Raw));
            return FixMath.F64.FromF32(F32_01);
        }

        /// <summary>
        /// Returns a float randomly distributed between lowerBound and upperBound
        /// </summary>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <returns></returns>
        public static FixMath.F64 Random(FixMath.F64 lowerBound, FixMath.F64 upperBound)
        {
            return lowerBound + (Random() * (upperBound - lowerBound));
        }

        public static int RandomInt(int min, int max)
        {
            return min + FixMath.F64.FloorToInt(Random() * (max - min));
        }
    }
}
