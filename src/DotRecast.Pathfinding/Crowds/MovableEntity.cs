using System.Diagnostics;
using System;
using SharpSteer2;
using SharpSteer2.Database;
using DotRecast.Pathfinding.Util;
using System.Collections.Generic;
using System.Numerics;
using SharpSteer2.Pathway;
using SharpSteer2.Helpers;
using SharpSteer2.Obstacles;

namespace DotRecast.Pathfinding.Crowds
{
    public class TemplateMovableEntity
    {
        // maximum move speed
        public float MaxSpeed = 6.0f;
        // maximum force
        public float MaxForce = 27.0f;
        // query local boundary radius
        public float QueryLocalBoundaryRadius = 5.0f;
        // query local neighbors radius
        public float QueryLocalNeighborRadius = 5.0f;
        //
        public float FollowPathAheadTime = 3.0f;
        public float FollowPathWeight = 1.0f;

        public float AvoidObstacleAheadTime = 0.1f;
        public float AvoidObstacleWeight = 1.0f;

        public float AvoidNeighborAheadTime = 1.0f;
        public float AvoidNeighborWeight = 1.0f;
    }

    public class MovableEntity : SimpleVehicle
    {
        // 单位状态
        public enum  eEntityState
        {
            Idle = 0,
	        Moving,
        };

        // a pointer to this boid's interface object for the proximity database
        private ITokenForProximityDatabase<IVehicle> _proximityToken;

        // Config values
        public override float MaxForce { get { return Template.MaxForce; } }
        public override float MaxSpeed { get { return Template.MaxSpeed; } }

        public TemplateMovableEntity Template { get; set; }

        public IMovableEntityManager EntityManager { get; private set; }

        public ILocalBoundaryQuerier LocalBoundaryQuerier { get; private set; }

        public IPathwayQuerier PathwayQuerier { get; private set; }
        
        // Entity unique id
        public UniqueId ID { get; set; }

        // Move destination
        public Vector3 TargetLocation { get; set; }

        public MovableEntityDebuger Debuger { get; private set; }

        // route for path following (waypoints and legs)
        private PolylinePathway Pathway = null;
        // True means walking forward along the path, false means walking backward along the path
        private bool _pathDirection = true;
        private Vector3? _pathReferencePosition = null;

        // local boundary
        public static readonly int  MaxBoundarySegmentNum = 10;
        private BoundarySegement[]  _boundarySegements = null;
        private int                 _boundarySegmentNum = 0;

        // local neighbors
        private List<IVehicle>      _neighbors = new List<IVehicle>(); 

        // state bits
        private uint                _stateBitsValue = 0;

        public void SetEntityState(eEntityState inState) { _stateBitsValue |= (uint)(1 << (int)inState); }
        public void ClearEntityState(eEntityState inState) { _stateBitsValue &= ~(uint)(1 << (int)inState); }
        public bool HasEntityState(eEntityState inState) { return (_stateBitsValue & (uint)(1 << (int)inState)) > 0; }

        // constructor
        public MovableEntity(IMovableEntityManager manager, 
                             IPathwayQuerier pathwayQuerier = null, 
                             ILocalBoundaryQuerier bounaryQuerier = null, 
                             IAnnotationService annotations = null)
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

        public List<IVehicle> FindNeighbors(float inRadius)
        {
            if (null == _proximityToken) return null;
            
            var result = new List<IVehicle>();
            _proximityToken.FindNeighbors(Position, inRadius, result);
            return result;
        }

        public virtual void OnCreate()
        {
            _boundarySegements = new BoundarySegement[MaxBoundarySegmentNum];
            Array.Fill(_boundarySegements, default(BoundarySegement));
            _boundarySegmentNum = 0;

            Reset();

#if ENABLE_STEER_AGENT_DEBUG
            Debuger = new MovableEntityDebuger();
#endif
        }

        public virtual void OnDelete()
        {
            Debuger = null;
        }

        public virtual void OnUpdate(float inDeltaTime)
        {
            if (!HasEntityState(eEntityState.Moving)) return;

            // update pathway
            if (null != PathwayQuerier)
            {
                Pathway = PathwayQuerier.FindPath(this);
            }

            // update neighbors
            if (null != _proximityToken)
            {
                _neighbors.Clear();
                _proximityToken.FindNeighbors(Position, Template.QueryLocalNeighborRadius, _neighbors);
            }

            // update local boundary
            if (null != LocalBoundaryQuerier)
            {
                _boundarySegmentNum = LocalBoundaryQuerier.QueryBoundaryInCircle(Template.QueryLocalBoundaryRadius, _boundarySegements, MaxBoundarySegmentNum);
            }

            // determine steering force
#if ENABLE_STEER_AGENT_DEBUG
            ref var info = ref Debuger.EntityDebugInfoBuff.Alloc(EntityManager.FrameNo);
            info.maxSpeed = MaxSpeed;
            info.radius = Radius;
            info.position = Position;
            info.velocity = Velocity;
            info.forward = Forward;
            info.side = Side;
            info.up = Up;
            info.steerPosition = _pathReferencePosition?? TargetLocation;
#endif

            var steerForce = determineCombinedSteering(inDeltaTime);
            if (steerForce != Vector3.Zero)
            {
                ApplySteeringForce(steerForce, inDeltaTime);

                if (null != _proximityToken)
                    _proximityToken.UpdateForNewPosition(Position);
            }
        }

        // compute combined steering force: move forward, avoid obstacles
        // or neighbors if needed, otherwise follow the path and wander
        Vector3 determineCombinedSteering(float elapsedTime)
        {
            if (null == EntityManager)
                return Vector3.Zero;

            const float forceScale = 1.0f;
#if ENABLE_STEER_AGENT_DEBUG
	        ref var info = ref Debuger.SteerFoceInfoBuff.Alloc(EntityManager.FrameNo);
	        info.reset();
	        info.position = Position;
	        info.maxForce = MaxForce;
#endif

            // probability that a lower priority behavior will be given a
            // chance to "drive" even if a higher priority behavior might
            // otherwise be triggered.
            const float leakThrough = 0.1f;

            // steering to seek target
            Vector3 steeringForce = Vector3.Zero;
            Vector3? referencePoint = null;
            // path follow corner point
            if (null != Pathway)
            {
                var predictionTime = Template.FollowPathAheadTime;
                steeringForce = this.SteerToFollowPath(_pathDirection, predictionTime, Pathway, MaxSpeed, out var currentPathDistance, annotation) * Template.FollowPathWeight;

                // float pathDistanceOffset = (_pathDirection ? 1 : -1) * predictionTime * Speed;
                referencePoint = Pathway.MapPathDistanceToPoint(currentPathDistance + 0.5f);
            }
            else
            {
                steeringForce = SteerForSeek(TargetLocation) * Template.FollowPathWeight;
            }
            steeringForce = steeringForce.TruncateLength(MaxForce);
            _pathReferencePosition = referencePoint;

#if ENABLE_STEER_AGENT_DEBUG
            info.targetForce = steeringForce * forceScale;
#endif

            // avoid obstacles
            var obstacles = new List<IObstacle>();
            Vector3 obstacleAvoidance = Vector3.Zero;
            if (leakThrough < RandomHelpers.Random())
            {
                for (var i = 0; i < _boundarySegmentNum; ++i)
                {
                    var boundary = _boundarySegements[i];
                    var v = boundary.End - boundary.Start;
                    v.Y = 0.0f;
                    var s = v.Normalize();
                    var f = Vector3Helpers.Up.Cross(s);
                    var u = f.Cross(s);
                    var p = (boundary.End + boundary.Start) * 0.5f;

                    var w = v.Length2D();
                    var h = 1.0f;
                    var obstacle = new RectangleObstacle(w, h, s, u, f, p, seenFromState.outside);
                    obstacles.Add(obstacle);

                }

                if (obstacles.Count > 0)
                {
                    if (referencePoint != null)
                    {
                        obstacleAvoidance = SteerToAvoidObstacles(Template.AvoidObstacleAheadTime, obstacles, referencePoint) * Template.AvoidObstacleWeight;
                    }
                    else
                    {
                        obstacleAvoidance = SteerToAvoidObstacles(Template.AvoidObstacleAheadTime, obstacles, null) * Template.AvoidObstacleWeight;
                    }
#if ENABLE_STEER_AGENT_DEBUG
			        info.obstacleForce = obstacleAvoidance * forceScale;
#endif
                }
            }

            // if obstacle avoidance is needed, do it
            if (obstacleAvoidance != Vector3.Zero)
            {
                steeringForce += obstacleAvoidance;
            }
            else
            {
                // otherwise consider avoiding collisions with others
                List<IVehicle> neighbors = new List<IVehicle>();
                Vector3 collisionAvoidance = Vector3.Zero;
                if (leakThrough < RandomHelpers.Random())
                {
                    for (int i = 0; i < _neighbors.Count; ++i)
                    {
                        var neighbor = _neighbors[i] as MovableEntity;
                        if (null == neighbor || ID == neighbor.ID) continue;
                        if (neighbor.HasEntityState(eEntityState.Moving)) continue;
                        neighbors.Add(neighbor);
                    }

                    // collisionAvoidance = steerToAvoidNeighbors(timeCollisionWithNeighbor, neighbors);
                    collisionAvoidance = SteerToAvoidNeighbors(Template.AvoidNeighborAheadTime, neighbors);
                    collisionAvoidance = collisionAvoidance.Normalize() * MaxForce * Template.AvoidNeighborWeight;

#if ENABLE_STEER_AGENT_DEBUG
			        info.avoidNeighborFoce = collisionAvoidance * forceScale;
#endif
                }

                // if collision avoidance is needed, do it
                if (collisionAvoidance != Vector3.Zero)
                {
                    steeringForce += collisionAvoidance;
                }
                else
                {
                    // do (interactively) selected type of path following
                    //const float pfLeadTime = 3;
                    //const Vec3 pathFollow =
                    //    (gUseDirectedPathFollowing ?
                    //        steerToFollowPath(pathDirection, pfLeadTime, *path) :
                    //        steerToStayOnPath(pfLeadTime, *path));

                    //// add in to steeringForce
                    //steeringForce += pathFollow * 0.5;
                }
            }

            var totalForce = steeringForce.SetYtoZero() * forceScale;
#if ENABLE_STEER_AGENT_DEBUG
	        info.steerForce = totalForce;
#endif

            return totalForce;
        }
    }
}
