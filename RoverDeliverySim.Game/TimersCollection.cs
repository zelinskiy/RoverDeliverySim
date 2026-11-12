using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace RoverDeliverySim;


using System.Collections;
using System.Collections.Concurrent;

public sealed class TimerCollection<TKey, TArg>(TimeSpan defaultInterval) : IEnumerable<TKey>
    where TKey : notnull
{
    private sealed class Entry(TimeSpan interval, Func<TArg, Task>? handler)
    {
        public readonly TimeSpan DefaultInterval = interval;
        public readonly Func<TArg, Task>? Handler = handler;
        public TimeSpan Interval = interval;
        public TimeSpan NextDue;
    }

    private readonly Dictionary<TKey, Entry> entries = [];
    private readonly ConcurrentQueue<(TKey Key, long IntervalUs)> pending = new();
    private TimeSpan lastNow;
    public TimeSpan DefaultInterval { get; } = defaultInterval;
    public TimeSpan GetInterval(TKey k) => entries[k].Interval;

    public void Add(TKey key, Func<TArg, Task>? handler = null)
        => Add(key, DefaultInterval, handler);

    public void Add(TKey key, TimeSpan interval, Func<TArg, Task>? handler = null)
        => entries.Add(key, new Entry(interval, handler));

    public int Count => entries.Count;
    public bool Contains(TKey key) => entries.ContainsKey(key);

    public IEnumerator<TKey> GetEnumerator() => entries.Keys.GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    public void PostIntervalMicroseconds(TKey key, long intervalUs)
        => pending.Enqueue((key, intervalUs));

    public async Task TickAsync(TimeSpan now, TArg arg)
    {
        lastNow = now;

        while (pending.TryDequeue(out var r))
            ApplyIntervalMicroseconds(r.Key, r.IntervalUs);

        foreach (var e in entries.Values)
        {
            if (e.Handler is null || e.Interval <= TimeSpan.Zero || now < e.NextDue) continue;

            // Keep the phase, but after a lag spike restart from now instead of firing catch-up bursts.
            var next = e.NextDue + e.Interval;
            e.NextDue = next > now ? next : now + e.Interval;

            await e.Handler(arg);
        }
    }

    private void ApplyIntervalMicroseconds(TKey key, long intervalUs)
    {
        if (!entries.TryGetValue(key, out var e)) return;

        e.Interval = intervalUs switch
        {
            < 0 => TimeSpan.Zero,
            0 => e.DefaultInterval,
            _ => TimeSpan.FromTicks(intervalUs * 10),
        };
        e.NextDue = lastNow + e.Interval;
    }
}