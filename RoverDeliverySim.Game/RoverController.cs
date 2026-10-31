using System;
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

        private RigidbodyComponent rigidBody;
        public Vector3 TargetPosition { get; set; } = Vector3.Zero;

        public override void Start()
        {
            rigidBody = Entity.Get<RigidbodyComponent>();
            if (rigidBody != null)
            {
                rigidBody.AngularFactor = new Vector3(0, 1, 0);
            }
        }

        public override void Update()
        {
            if (TargetPosition != Vector3.Zero)
            {
                RideTo(TargetPosition);
            }
            else
            {
                HandleInput();
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