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

        public override void Start()
        {
            rigidBody = Entity.Get<RigidbodyComponent>();
            rigidBody.AngularFactor = new Vector3(0, 1, 0);
        }

        public override void Update()
        {
            float moveInput = 0.0f;
            float turnInput = 0.0f;

            if (Input.IsKeyDown(Keys.W)) moveInput += 1.0f;
            if (Input.IsKeyDown(Keys.S)) moveInput -= 1.0f;
            if (Input.IsKeyDown(Keys.A)) turnInput += 1.0f;
            if (Input.IsKeyDown(Keys.D)) turnInput -= 1.0f;

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
}