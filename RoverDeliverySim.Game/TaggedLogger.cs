using Stride.Core.Diagnostics;
using System;
using System.Diagnostics;

namespace RoverDeliverySim;

public class TaggedLogger(string componentName)
{
    public string ComponentName { get; } = componentName;
    private readonly Logger logger = GlobalLogger.GetLogger(componentName);

    public void Info(string message)
        => logger.Info($"[{DateTime.Now:HH:mm:ss.fff}] {message}");

    public void Warning(string message)
        => logger.Warning($"[{DateTime.Now:HH:mm:ss.fff}] {message}");

    public void Error(string message)
        => logger.Error($"[{DateTime.Now:HH:mm:ss.fff}] {message}");

    /// <summary>
    /// Times a block of code and automatically logs how long it took in milliseconds.
    /// </summary>
    public IDisposable Measure(string operationName)
    {
        return new OperationTimer(this, operationName);
    }

    private class OperationTimer : IDisposable
    {
        private readonly TaggedLogger log;
        private readonly string operationName;
        private readonly Stopwatch stopwatch;

        public OperationTimer(TaggedLogger log, string operationName)
        {
            this.log = log;
            this.operationName = operationName;
            log.Info($"Started: {operationName}");
            stopwatch = Stopwatch.StartNew();
        }

        public void Dispose()
        {
            stopwatch.Stop();
            log.Info($"Completed: {operationName} in {stopwatch.ElapsedMilliseconds}ms");
        }
    }
}