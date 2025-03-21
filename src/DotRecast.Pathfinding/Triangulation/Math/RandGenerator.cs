using System;
using System.Collections.Generic;

namespace Pathfinding.Triangulation.Math
{
    public class RandGenerator
    {
        public RandGenerator(int seed = 1234, int rangeMin_ = 0, int rangeMax_ = 1)
        {
            _rangeMin = rangeMin_;
            _rangeMax = rangeMax_;

            _rand = new Random(seed);
        }

        public int _rangeMin;

        public int _rangeMax;

        public Random _rand;

        public int set_seed(int seed)
        {
            _rand = new Random(seed);
            return seed;
        }


        public int next()
        {
            var F32_01 = FixMath.F32.FromRaw(_rand.Next(FixMath.F32.One.Raw));
            var factor = FixMath.F64.FromF32(F32_01);
            int res = FixMath.F64.RoundToInt(_rangeMin + factor * (_rangeMax - _rangeMin));
            return res;
        }


        public int nextInRange(int rangeMin, int rangeMax)
        {
            this._rangeMin = rangeMin;
            this._rangeMax = rangeMax;
            return this.next();
        }


        public void shuffle<T>(List<T> array)
        {
            int currIdx = array.Count;
            while ((currIdx > 0))
            {
                int rndIdx = this.nextInRange(0, (currIdx - 1));
                --currIdx;
                T tmp = array[currIdx];
                array[currIdx] = array[rndIdx];
                array[rndIdx] = tmp;
            }
        }
    }
}
