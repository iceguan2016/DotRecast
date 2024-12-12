using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace SharpSteer2.Obstacles
{
    public class ObstacleGroup : IEnumerable<IObstacle>
    {
        private List<IObstacle> items = new List<IObstacle>();

        public int Count => items.Count;

        public void Add(IObstacle obstacle) { items.Add(obstacle); }

        public IEnumerator<IObstacle> GetEnumerator()
        {
            return new ObstacleGroupEnumerator(this);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return new ObstacleGroupEnumerator(this);
        }

        private class ObstacleGroupEnumerator : IEnumerator<IObstacle>
        {
            ObstacleGroup group;
            int index;
            IObstacle currentElement;
            public ObstacleGroupEnumerator(ObstacleGroup _group)
            {
                group = _group;
                index = -1;
            }

            public object Current => currentElement;

            IObstacle IEnumerator<IObstacle>.Current => currentElement;

            public void Dispose()
            {
                index = -1;
                group = null;
                currentElement = null;
            }

            public bool MoveNext()
            {
                if (index < group.items.Count - 1)
                {
                    currentElement = group.items[++index];
                    return true;
                }
                else
                {
                    currentElement = null;
                    return false;
                }
            }

            public void Reset()
            {
                index = -1;
            }
        }
    }
}
