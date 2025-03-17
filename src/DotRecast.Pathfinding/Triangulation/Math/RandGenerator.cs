using System;
using System.Collections.Generic;

namespace Pathfinding.Triangulation.Math
{
    public class RandGenerator
    {
        public RandGenerator(int seed = 1234, int rangeMin_ = 0, int rangeMax_ = 1)
        {
            _originalSeed = _currSeed = seed;
            rangeMin = rangeMin_;
            rangeMax = rangeMax_;

            _numIter = 0;
        }
        
        public int rangeMin;

        public int rangeMax;

        public int _originalSeed;

        public int _currSeed;

        public int _rangeMin;

        public int _rangeMax;

        public int _numIter;

        public string _tempString;

        public int set_seed(int @value)
        {
            this._originalSeed = this._currSeed = @value;
            return @value;
        }


        public int get_seed()
        {
            return this._originalSeed;
        }


        public void reset()
        {
            this._currSeed = this._originalSeed;
            this._numIter = 0;
        }


        public int next()
        {
            long sq = _currSeed * _currSeed;
            _tempString = sq.ToString();
            while (_tempString.Length < 8)
            {
                _tempString = "0" + _tempString;
            }

            _currSeed = int.Parse(_tempString.Substring(1, 5));
            var factor = FixMath.F64.FromInt(_currSeed) / 99999;
            int res = FixMath.F64.RoundToInt(rangeMin + factor * (rangeMax - rangeMin));
            if (_currSeed == 0)
            {
                _currSeed = _originalSeed + _numIter;
            }

            _numIter++;
            if (_numIter == 200)
            {
                reset();
            }

            return res;
        }


        public int nextInRange(int rangeMin, int rangeMax)
        {
            this.rangeMin = rangeMin;
            this.rangeMax = rangeMax;
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
