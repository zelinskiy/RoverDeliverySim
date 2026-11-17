using Asv.Mavlink;
using DotNext.Collections.Generic;
using R3;
using RoverDeliverySim.MavlinkExtensions;
using Stride.Core;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Physics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

namespace RoverDeliverySim;

public class RoverController : SyncScript
{

    public enum RoverMode
    {
        Manual,
        Hold,
        Guided,
        Auto,
        RTL,
    }

    public float MoveSpeed { get; set; } = 10.0f;
    public float TurnSpeed { get; set; } = 4.0f;
    // TODO: Navigate along predefined safe routes
    public Entity NavNetwork { get; set; }
    private List<Entity> NavNetworkPoints { get; set; }
    public Entity MissionWaypointTemplate { get; set; }
    [DataMemberIgnore]
    public RoverMission? Mission;
    private RoverMission? SmartRtlMission;
    // Link to RigidbodyComponent.MissionBusAdapter
    [DataMemberIgnore]
    public MissionBus MissionBusAdapter;
    public readonly float WaypointReachedTolerance = 0.25f;
    private new readonly TaggedLogger Log = new("RoverController");
    private RigidbodyComponent _rigidBody;
    // TODO: mk method to hide from editor
    [DataMemberIgnore]
    public Vector2? TargetPosition { get; set; } = null;
    public RoverMode Mode { get; private set; } = RoverMode.Hold;

    public override void Start()
    {
        _rigidBody = Entity.Get<RigidbodyComponent>();
        if (_rigidBody != null)
        {
            _rigidBody.AngularFactor = new Vector3(0, 1, 0);
        }
        NavNetworkPoints = NavNetwork.GetChildren().ToList();
        if (NavNetworkPoints.Count < 2)
        {
            throw new ArgumentException("Navigation Network shall have at least 2 points");
        }
        MissionBusAdapter = Entity.Get<RoverMavLinkConnection>().MissionBusAdapter;
        MissionBusAdapter.FromConnection.Subscribe(ConnectionMessageHandler);            
        //RunTestMission();
    }

    private void RunTestMission()
    {            
        var waypoints = NavNetworkPoints.Select(p => new Vector2(p.Transform.Position.X, p.Transform.Position.Z));
        waypoints = Enumerable.Repeat(waypoints, 3).SelectMany(wp => wp);
        var mission = new RoverMission(waypoints);
        // To remove interference with MissionServer in RoverMavlinkConnection
        MissionBusAdapter = new MissionBus();
        MissionBusAdapter.FromConnection.Subscribe(ConnectionMessageHandler);
        MissionBusAdapter.Send(new ConnectionMsg.LoadMission(mission));
        MissionBusAdapter.Send(new ConnectionMsg.StartMission());
    }

    private void ConnectionMessageHandler(ConnectionMsg msg)
    {
        List<Type> missionCommands = [
            typeof(ConnectionMsg.StartMission),
            typeof(ConnectionMsg.PauseMission),
            typeof(ConnectionMsg.FinishMission),
            typeof(ConnectionMsg.CancelMission),
            typeof(ConnectionMsg.ClearMission),
            typeof(ConnectionMsg.ChangeWaypoint),
        ];

        if(Mission is null && missionCommands.Contains(msg.GetType()))
        {
            Log.Warning($"Trying to execute {msg} without a set mission");
            return;
        }
        switch (msg)
        {
            case ConnectionMsg.LoadMission payload:
                Log.Info("Load mission");
                Mission = payload.Mission;
                Mission.Reset();
                return;
            case ConnectionMsg.StartMission:
                Log.Info("Start mission");                    
                Mission.Start();
                TargetPosition = Mission.Target;
                return;
            case ConnectionMsg.PauseMission:
                Log.Info("Mission paused");
                Mission.Pause();
                TargetPosition = null;
                return;
            case ConnectionMsg.FinishMission:
                Log.Info("Mission finished");
                Mission.Finish();
                TargetPosition = null;
                return;
            case ConnectionMsg.CancelMission:
                Log.Info("Mission cancelled");
                Mission.Cancel();
                TargetPosition = null;
                return;
            case ConnectionMsg.ClearMission:
                Log.Info($"Mission cleared");
                Mission = null;
                TargetPosition = null;
                return;
            case ConnectionMsg.ChangeWaypoint payload:
                Log.Info($"Moving to waypoint {payload.Seq}");
                Mission.SetTarget(payload.Seq);
                TargetPosition = Mission.Target;
                return;                
            case ConnectionMsg.SetMode payload:
                var mode = payload.Mode;
                Log.Info($"Changing mode to {mode}");
                Mode = mode;
                if(mode == RoverMode.RTL)
                {
                    if(Mission is not null)
                    {
                        Log.Info($"Executing Smart RTL mission");
                        Mission.Cancel();
                        SmartRtlMission = Mission.GetSmartRtlMission();
                        SmartRtlMission.Start();
                        return;
                    }
                    else
                    {
                        Log.Warning($"Currently can't execute Smart RTL without a previously set mission");
                        Mode = RoverMode.Hold;
                        return;
                    }
                }
                else if(mode == RoverMode.Auto && Mission is not null)
                {
                    Mission.Start();
                    return;
                }
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
        else if (TargetPosition is Vector2 target)
        {
            RideTo(target);
        }
        else
        {
            HandleInput();
        }
    }

    private void HandleMission()
    {
        if (Mission is null || Mission.Status is not MissionStatus.Running)
        {
            return;
        }

        var position = Entity.Transform.Position.XZ();
        bool reachedLast = Vector2.Distance(position, Mission.Waypoints.Last()) 
                < WaypointReachedTolerance && Mission.Seq == Mission.Waypoints.Count - 1;  
        if (reachedLast)
        {
            var response = new ControllerMsg.WaypointReached(Mission.Seq);
            MissionBusAdapter.Send(response);
            Mission.Finish();
            TargetPosition = null;
            return;
        }

        if (TargetPosition is Vector2 target)
        {
            if(Vector2.Distance(position, target) < WaypointReachedTolerance)
            {
                var response = new ControllerMsg.WaypointReached(Mission.Seq);
                MissionBusAdapter.Send(response);
                TargetPosition = Mission.TargetNextWaypoint();
                if(Mode == RoverMode.Guided)
                {
                    Mission.Pause();
                    return;
                }
            }
            RideTo(target);
            return;
        }
        
    }

    private void HandleSmartRtlMission()
    {
        if (SmartRtlMission is null)
        {
            return;
        }
        if (TargetPosition is not null)
        {
            RideTo(SmartRtlMission.Target);
        }
        else
        {
            TargetPosition = SmartRtlMission.TargetNextWaypoint();
        }
        if (Vector2.Distance(Entity.Transform.Position.XZ(), SmartRtlMission.Waypoints.Last()) < WaypointReachedTolerance)
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

    private void RideTo(Vector2 targetPosition)
    {
        const float angleTolerance = 0.98f;

        Vector3 pos = Entity.Transform.Position;

        Vector3 toTarget = new(targetPosition.X - pos.X, 0f, targetPosition.Y - pos.Z);
        float distance = toTarget.Length();

        // Arrived at point
        if (distance < WaypointReachedTolerance)
        {
            _rigidBody.LinearVelocity = new Vector3(0, _rigidBody.LinearVelocity.Y, 0);
            _rigidBody.AngularVelocity = Vector3.Zero;
            return;
        }
        toTarget /= distance;

        Vector3 currentDirection = Entity.Transform.WorldMatrix.Forward;
        currentDirection.Y = 0f;
        if (currentDirection.LengthSquared() < 1e-6f) return;
        currentDirection.Normalize();

        float dot = Vector3.Dot(currentDirection, toTarget);

        if (dot > angleTolerance)
        {
            // Facing the target: drive forward
            _rigidBody.AngularVelocity = Vector3.Zero;
            Vector3 driveVel = currentDirection * MoveSpeed;
            driveVel.Y = _rigidBody.LinearVelocity.Y;
            _rigidBody.LinearVelocity = driveVel;
        }
        else
        {
            // Turn in place
            Vector3 cross = Vector3.Cross(currentDirection, toTarget);
            _rigidBody.LinearVelocity = new Vector3(0, _rigidBody.LinearVelocity.Y, 0);
            Vector3 angularVel = _rigidBody.AngularVelocity;
            // Target exactly behind: cross.Y == 0, so pick a side
            angularVel.Y = (cross.Y >= 0 ? 1f : -1f) * TurnSpeed;
            _rigidBody.AngularVelocity = angularVel;
        }
    }
}