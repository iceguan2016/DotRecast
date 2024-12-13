using System.Diagnostics;
using System;
using SharpSteer2;
using SharpSteer2.Database;
using DotRecast.Pathfinding.Util;

namespace DotRecast.Pathfinding.Crowds
{
    public class MovableEntity : SimpleVehicle
    {
        // 单位状态
        enum  eEntityState
        {
            Idle = 0,
	        Moving,
        };

        // a pointer to this boid's interface object for the proximity database
        private ITokenForProximityDatabase<IVehicle> _proximityToken;

        public override float MaxForce { get { return 27; } }
        public override float MaxSpeed { get { return 9; } }

        public IMovableEntityManager EntityManager { get; private set; }

        public UniqueId ID { get; set; }

        // constructor
        public MovableEntity(IMovableEntityManager manager, IProximityDatabase<IVehicle> pd, IAnnotationService annotations = null)
            : base(annotations)
        {
            EntityManager = manager;
        }

        // reset state
        public override void Reset()
        {
            // reset the vehicle
            base.Reset();

            // initial slow speed
            Speed = (MaxSpeed * 0.3f);

            // randomize initial orientation
            //RegenerateOrthonormalBasisUF(Vector3Helpers.RandomUnitVector());
            //Vector3 d = Vector3Helpers.RandomUnitVector();
            //d.X = Math.Abs(d.X);
            //d.Y = 0;
            //d.Z = Math.Abs(d.Z);
            //RegenerateOrthonormalBasisUF(d);

            //// randomize initial position
            //Position = Vector3.UnitX * 10 + (Vector3Helpers.RandomVectorInUnitRadiusSphere() * 20);

            // notify proximity database that our position has changed
            //FIXME: SimpleVehicle::SimpleVehicle() calls reset() before proximityToken is set
            if (_proximityToken != null)
                _proximityToken.UpdateForNewPosition(Position);
        }

        public void NewPD(IProximityDatabase<IVehicle> pd)
        {
            // delete this boid's token in the old proximity database
            if (_proximityToken != null)
            {
                _proximityToken.Dispose();
                _proximityToken = null;
            }

            // allocate a token for this boid in the proximity database
            if(null != pd) _proximityToken = pd.AllocateToken(this);
        }

        public virtual void OnCreate()
        {
            Reset();
        }

        public virtual void OnDestroy()
        {
        }
    }
}
