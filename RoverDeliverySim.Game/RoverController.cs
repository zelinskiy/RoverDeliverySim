using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Physics;

namespace RoverDeliverySim
{
    public class RoverController : SyncScript
    {
        public float MoveSpeed { get; set; } = 10.0f;
        public float TurnSpeed { get; set; } = 4.0f;
        public Entity NavNetwork { get; set; }
        private List<Entity> NavNetworkPoints { get; set; }
        public Entity MissionWaypointTemplate { get; set; }
        public RoverMission Mission;
        
        

        private RigidbodyComponent rigidBody;
        // TODO: mk method to hide from editor
        public Vector3 TargetPosition { get; set; } = Vector3.Zero;

        public override void Start()
        {
            rigidBody = Entity.Get<RigidbodyComponent>();
            if (rigidBody != null)
            {
                rigidBody.AngularFactor = new Vector3(0, 1, 0);
            }
            //NavNetworkPoints = NavNetwork.GetChildren().ToList();
            //if (NavNetworkPoints.Count < 2)
            //{
            //    throw new ArgumentException("Navigation Network shall have at least 2 points");
            //}
        }

        public override void Update()
        {
            if (Mission is not null)
            {
                HandleMission();
            }
            
            //if (TargetPosition != Vector3.Zero)
            //{
            //    RideTo(TargetPosition);
            //}
            //else
            //{
            //    HandleInput();
            //    Navigate();
            //}
        }

        private int navPathDirection = 1;

        public Entity GetNextNavPoint()
        {
            var closestPoint = NavNetworkPoints.MinBy(p => Vector3.Distance(p.Transform.Position, Entity.Transform.Position));
            if (closestPoint == NavNetworkPoints.First())
            {
                navPathDirection = 1;
            }
            if (closestPoint == NavNetworkPoints.Last())
            {
                navPathDirection = -1;
            }
            var i = NavNetworkPoints.IndexOf(closestPoint);
            return NavNetworkPoints[i + navPathDirection];
        }

        public void Navigate()
        {
            if(TargetPosition != Vector3.Zero)
            {
                RideTo(TargetPosition);
            } 
            else
            {
                TargetPosition = GetNextNavPoint().Transform.Position;
            }
        }
        private void HandleMission()
        {
            // TODO: Tidy & refactor this whole condition
            if (Mission is null || Mission.Status is MissionStatus.Finished or MissionStatus.Failed or MissionStatus.Paused or MissionStatus.Cancelled)
            {
                return;
            }
            else if (TargetPosition != Vector3.Zero)
            {
                RideTo((Vector3)Mission.Target);
            }
            else if(Mission.Running && Vector3.Distance(Entity.Transform.Position, (Vector3)Mission.Waypoints.Last()) < 0.3f)
            {
                Mission.Status = MissionStatus.Finished;
            }
            else
            {
                TargetPosition = (Vector3)Mission.SetNextWaypoint();
            }
        }

        private void HandleInput()
        {
            float moveInput = 0.0f;
            float turnInput = 0.0f;

            if (Input.IsKeyDown(Keys.W)) moveInput += 1.0f;
            if (Input.IsKeyDown(Keys.S)) moveInput -= 1.0f;
            if (Input.IsKeyDown(Keys.A)) turnInput += 1.0f;
            if (Input.IsKeyDown(Keys.D)) turnInput -= 1.0f;

            if (rigidBody != null)
            {
                Vector3 angularVel = rigidBody.AngularVelocity;
                angularVel.Y = turnInput * TurnSpeed;
                rigidBody.AngularVelocity = angularVel;

                Vector3 forwardDir = Entity.Transform.WorldMatrix.Forward;
                forwardDir.Normalize();

                Vector3 targetVelocity = forwardDir * (moveInput * MoveSpeed);
                targetVelocity.Y = rigidBody.LinearVelocity.Y; // Preserve gravity
                rigidBody.LinearVelocity = targetVelocity;
            }
        }


        private void RideTo(Vector3 targetPosition)
        {
            float distanceTolerance = 0.3f;
            float angleTolerance = 0.98f;
            Vector3 currentPosition = Entity.Transform.Position;

            // Arrived at point
            if (Vector3.Distance(currentPosition, targetPosition) < distanceTolerance)
            {
                TargetPosition = Vector3.Zero;
                rigidBody.LinearVelocity = new Vector3(0, rigidBody.LinearVelocity.Y, 0);
                rigidBody.AngularVelocity = Vector3.Zero;
                return;
            }

            Vector3 currentDirection = Entity.Transform.WorldMatrix.Forward;
            currentDirection.Y = 0f;
            currentDirection.Normalize();            
            Vector3 toTarget = targetPosition - currentPosition;
            toTarget.Y = 0f;
            toTarget.Normalize();

            float dot = Vector3.Dot(currentDirection, toTarget);

            if (dot > angleTolerance)
            {
                rigidBody.AngularVelocity = new Vector3(0, 0, 0);
                Vector3 driveVel = currentDirection * MoveSpeed;
                driveVel.Y = rigidBody.LinearVelocity.Y;
                rigidBody.LinearVelocity = driveVel;
            }
            else
            {
                Vector3 cross = Vector3.Cross(currentDirection, toTarget);
                rigidBody.LinearVelocity = new Vector3(0, rigidBody.LinearVelocity.Y, 0);
                Vector3 angularVel = rigidBody.AngularVelocity;
                angularVel.Y = Math.Sign(cross.Y) * TurnSpeed;
                rigidBody.AngularVelocity = angularVel;
            }           
        }
    }
}