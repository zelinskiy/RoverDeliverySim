using Asv.Mavlink;
using DotNext.Collections.Generic;
using R3;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Physics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace RoverDeliverySim
{
    public class RoverController : SyncScript
    {
        public float MoveSpeed { get; set; } = 10.0f;
        public float TurnSpeed { get; set; } = 4.0f;
        // TODO: Navigate along predefined safe routes
        public Entity NavNetwork { get; set; }
        private List<Entity> NavNetworkPoints { get; set; }
        public Entity MissionWaypointTemplate { get; set; }
        public RoverMission? Mission;
        private RoverMission? SmartRtlMission;
        // Link to RigidbodyComponent.MissionBusAdapter
        public MissionBus MissionBusAdapter;
        private new readonly Logger Log = GlobalLogger.GetLogger("Rover");
        private RigidbodyComponent _rigidBody;
        // TODO: mk method to hide from editor
        public Vector3? TargetPosition { get; set; }

        public override void Start()
        {
            _rigidBody = Entity.Get<RigidbodyComponent>();
            if (_rigidBody != null)
            {
                _rigidBody.AngularFactor = new Vector3(0, 1, 0);
            }
            //NavNetworkPoints = NavNetwork.GetChildren().ToList();
            //if (NavNetworkPoints.Count < 2)
            //{
            //    throw new ArgumentException("Navigation Network shall have at least 2 points");
            //}
            MissionBusAdapter = Entity.Get<RoverMavLinkConnection>().MissionBusAdapter;
            MissionBusAdapter.FromConnection.Subscribe(ConnectionMessageHandler);
        }

        private void ConnectionMessageHandler(ConnectionMsg msg)
        {
            if(Mission is null && msg is not ConnectionMsg.LoadMission)
            {
                Log.Warning($"Trying to execute {msg} without a set mission");
                return;
            }
            switch (msg)
            {
                case ConnectionMsg.LoadMission payload:
                    Mission = payload.Mission;
                    Mission.Reset();
                    return;
                case ConnectionMsg.StartMission:
                    Log.Info("Start mission");
                    Mission.Start();
                    return;
                case ConnectionMsg.PauseMission:
                    Log.Info("Mission paused");
                    Mission?.Pause();
                    return;
                case ConnectionMsg.FinishMission:
                    Log.Info("Mission finished");
                    Mission.Finish();
                    return;
                case ConnectionMsg.CancelMission:
                    Log.Info("Mission cancelled");
                    Mission.Cancel();
                    return;
                case ConnectionMsg.ClearMission:
                    Log.Info($"Mission cleared");
                    Mission = null;
                    return;
                case ConnectionMsg.ChangeWaypoint payload:
                    Log.Info($"Moving to waypoint {payload.Seq}");
                    Mission.SetTarget(payload.Seq);
                    return;
                case ConnectionMsg.SmartRtl:
                    Log.Info($"Executing Smart RTL mission");
                    Mission.Cancel();
                    SmartRtlMission = Mission.GetSmartRtlMission();
                    SmartRtlMission.Start();
                    return;
                default:
                    throw new NotImplementedException();
            }
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

        private void HandleMission()
        {
            if (Mission is null || Mission.Status is not MissionStatus.Running)
            {
                return;
            }
            else if (TargetPosition is not null)
            {
                RideTo((Vector3)Mission.Target);
                return;
            }            
            else if(Vector3.Distance(Entity.Transform.Position, (Vector3)Mission.Waypoints.Last()) < 0.3f)
            {
                var response = new ControllerMsg.WaypointReached(Mission.Seq);
                MissionBusAdapter.Send(response);
                Mission.Finish();
                return;
            }
            else
            {
                var response = new ControllerMsg.WaypointReached(Mission.Seq);
                MissionBusAdapter.Send(response);
                TargetPosition = (Vector3)Mission.TargetNextWaypoint();
                return;
            }
        }

        private void HandleSmartRtlMission()
        {
            if (SmartRtlMission is null)
            {
                return;
            }
            else if (TargetPosition != Vector3.Zero)
            {
                RideTo((Vector3)SmartRtlMission.Target);
            }
            else if (TargetPosition == Vector3.Zero)
            {
                TargetPosition = (Vector3)SmartRtlMission.TargetNextWaypoint();
            }
            else if (Vector3.Distance(Entity.Transform.Position, (Vector3)SmartRtlMission.Waypoints.Last()) < 0.3f)
            {
                // TODO: Notify we're home
                SmartRtlMission = null;
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

            if (_rigidBody != null)
            {
                Vector3 angularVel = _rigidBody.AngularVelocity;
                angularVel.Y = turnInput * TurnSpeed;
                _rigidBody.AngularVelocity = angularVel;

                Vector3 forwardDir = Entity.Transform.WorldMatrix.Forward;
                forwardDir.Normalize();

                Vector3 targetVelocity = forwardDir * (moveInput * MoveSpeed);
                targetVelocity.Y = _rigidBody.LinearVelocity.Y; // Preserve gravity
                _rigidBody.LinearVelocity = targetVelocity;
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
                _rigidBody.LinearVelocity = new Vector3(0, _rigidBody.LinearVelocity.Y, 0);
                _rigidBody.AngularVelocity = Vector3.Zero;
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
                _rigidBody.AngularVelocity = new Vector3(0, 0, 0);
                Vector3 driveVel = currentDirection * MoveSpeed;
                driveVel.Y = _rigidBody.LinearVelocity.Y;
                _rigidBody.LinearVelocity = driveVel;
            }
            else
            {
                Vector3 cross = Vector3.Cross(currentDirection, toTarget);
                _rigidBody.LinearVelocity = new Vector3(0, _rigidBody.LinearVelocity.Y, 0);
                Vector3 angularVel = _rigidBody.AngularVelocity;
                angularVel.Y = Math.Sign(cross.Y) * TurnSpeed;
                _rigidBody.AngularVelocity = angularVel;
            }           
        }
    }
}