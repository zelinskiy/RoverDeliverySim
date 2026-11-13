using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Stride.Core.Extensions;
using Stride.Core.Mathematics;

namespace RoverDeliverySim
{
    // TBD: Make this class private?
    public enum MissionStatus
    {
        Ready,
        Running,
        Paused,
        Finished,
        Cancelled,
        Failed,
    }
    public class RoverMission
    {
        public List<Vector2> Waypoints { get; }
        public ushort Seq { get; private set; }
        public Vector2 Target
        {
            get
            {
                return Waypoints.ElementAt(Seq);
            }
        }

        public MissionStatus Status { 
            get;
            private set;
        }

        public bool Finished => Status == MissionStatus.Finished;
        public bool Ready => Status == MissionStatus.Ready;
        public bool Running => Status == MissionStatus.Running;

        public RoverMission(IEnumerable<Vector2> waypoints)
        {
            if(waypoints.IsNullOrEmpty())
            {
                throw new ArgumentException("Mission must have at least one waypoint");
            }
            Waypoints = [..waypoints];
            Seq = 0;
            Status = MissionStatus.Ready;
        }

        public void SetTarget(int index)
        {
            if(index < 0 || index >= Waypoints.Count)
            {
                throw new ArgumentException($"Waypoint {index} is out of range");
            }
            Seq = (ushort)index;
        }

        public void Reset()
        {
            Status = MissionStatus.Ready;
            Seq = 0;
        }        
        public void Start() => Status = MissionStatus.Running;
        
        public void Pause() => Status = MissionStatus.Paused;
        public void Cancel() => Status = MissionStatus.Paused;
        public void Abort() => Status = MissionStatus.Failed;
        public void Finish() => Status = MissionStatus.Finished;

        public Vector2 TargetNextWaypoint() 
        {
            if (Seq == Waypoints.Count)
            {
                throw new InvalidOperationException("Already heading to the last waypoint");
            }
            Seq++;
            return Target;
        }

        public RoverMission GetSmartRtlMission()
        {
            return new RoverMission(Waypoints.Take(Seq + 1).Reverse().ToList());
        }

        
    }
}
