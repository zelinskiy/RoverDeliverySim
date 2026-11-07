using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
        private int _targetIndex;
        public Vector2 Target
        {
            get
            {
                return Waypoints.ElementAt(_targetIndex);
            }
        }

        public MissionStatus Status { 
            get; 
            set
            {
                if (value == MissionStatus.Ready)
                {
                    //throw new ArgumentException()
                }
                field = value;
            }
        }

        public bool Finished => Status == MissionStatus.Finished;
        public bool Ready => Status == MissionStatus.Ready;
        public bool Running => Status == MissionStatus.Running;

        public RoverMission(IEnumerable<Vector2> waypoints)
        {
            if(waypoints.Any())
            {
                throw new ArgumentException("Mission must have at least one waypoint");
            }
            Waypoints = [..waypoints];
            _targetIndex = 0;
            Status = MissionStatus.Ready;
        }

        public void SetTarget(int index)
        {
            if(index < 0 || index >= Waypoints.Count)
            {
                throw new IndexOutOfRangeException($"Waypoint {index} is out of range");
            }
            _targetIndex = index;
        }

        public void Reset()
        {
            Status = MissionStatus.Ready;
            _targetIndex = 0;
        }

        public void Start()
        {
            Status = MissionStatus.Running;
        }
        
        public Vector2 SetNextWaypoint() 
        {
            if (_targetIndex == Waypoints.Count)
            {
                throw new InvalidOperationException("Already heading to the last waypoint");
            }
            _targetIndex++;
            return Target;
        }
    }
}
