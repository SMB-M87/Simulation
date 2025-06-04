using Akka.Actor;
using Akka.Event;
using Procedure = Simulation.Dummy.Procedure;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace Simulation.Tick
{
    /// <summary>
    /// Manages the simulation update loop responsible for maintaining a consistent updates-per-second (UPS) rate.
    /// The update loop runs in a background thread and sends update messages to an Akka.NET actor at a configurable interval.
    /// It supports pausing, dynamic UPS adjustments and tracking the effective UPS over time.
    /// </summary>
    internal class UpdateCycle
    {
        /// <summary>
        /// Reference to the <see cref="CycleActor"/> that receives <see cref="UpdateTick"/> messages to trigger simulation updates.
        /// </summary>
        private readonly IActorRef _instance;

        /// <summary>
        /// Indicates whether the update loop is currently running. Marked as volatile to ensure thread-safe reads and writes.
        /// </summary>
        public volatile bool Running;

        /// <summary>
        /// Update interval in milliseconds, calculated from the target UPS (e.g., 1000 / UPS).
        /// Determines how often update ticks are sent.
        /// </summary>
        private float _upsMS;

        /// <summary>
        /// Number of update ticks performed in the current one-second interval. Used to calculate effective UPS.
        /// </summary>
        private uint _updateCount;

        /// <summary>
        /// Gets the current target UPS (updates per second) set for the update loop.
        /// </summary>
        internal uint TargetUPS { get; private set; }

        /// <summary>
        /// Gets the effective number of updates executed in the last second.
        /// </summary>
        internal uint CurrentUPS { get; private set; }

        /// <summary>
        /// Indicates whether the update loop is currently paused.
        /// The simulation starts in a paused state by default to allow the user to configure settings 
        /// without any update ticks being processed or logged while the simulation is idle.
        /// </summary>
        internal bool Paused { get; private set; } = true;

        /// <summary>
        /// Accumulates the total wall-clock time spent in the Paused state.
        /// </summary>
        internal Stopwatch PauseTimer { get; private set; } = new();

        /// <summary>
        /// Stopwatch used to track when the UPS interval was last manually adjusted via the UI.
        /// Prevents premature correction of the update interval before the system has had time to adapt to the new setting.
        /// </summary>
        internal Stopwatch AdjustedUPSTimer { get; private set; } = new();

        /// <summary>
        /// Synchronization lock used to safely send messages to the actor from the update thread.
        /// </summary>
        private readonly object _syncLock = new();

        /// <summary>
        /// Background thread in which the update loop executes.
        /// </summary>
        private Thread? _updateThread;

        /// <summary>
        /// Used to keep track of the starting time of the update loop.
        /// </summary>
        private bool _started = false;

        /// <summary>
        /// Initializes a new instance of the <see cref="UpdateCycle"/> class.
        /// </summary>
        /// <param name="instance">The Akka.NET actor that will receive update messages.</param>
        /// <param name="targetUPS">The desired number of updates per second.</param>
        internal UpdateCycle(IActorRef instance, uint targetUPS)
        {
            _instance = instance;
            _upsMS = 1000.0f / targetUPS;
            TargetUPS = targetUPS;
        }

        /// <summary>
        /// Starts the update loop in a background thread if it is not already running.
        /// Also starts the pause timer, which is used to track simulation inactivity for log purposes.
        /// </summary>
        internal void Start()
        {
            if (Running) return;

            Running = true;
            _updateThread = new Thread(UpdateLoop) { IsBackground = true };
            _updateThread.Start();

            _started = false;
            Paused = true;
            PauseTimer = new();

            App.Log.Info("[UpdateCycle] Initialized the update loop");
        }

        /// <summary>
        /// Stops the update loop by setting the running flag to false and joining the update thread.
        /// Also stops the pause timer to prevent further activity tracking.
        /// </summary>
        internal void Stop()
        {
            Running = false;
            _updateThread?.Join();
            PauseTimer.Stop();

            App.Log.Info("[UpdateCycle] Destroyed the update loop");
        }

        /// <summary>
        /// Toggles the paused state of the update loop.
        /// While paused, updates are not sent to the actor, also manages the PauseTimer to
        /// keep the actual runtime of the simulation accurate.
        /// </summary>
        internal void TogglePause()
        {
            if (!Paused)
            {
                Paused = true;
                PauseTimer.Start();
                App.Log.Info("[UpdateCycle] Paused the update loop");
            }
            else
            {
                PauseTimer.Stop();
                AdjustedUPSTimer.Restart();
                Paused = false;

                if (_started)
                    App.Log.Info("[UpdateCycle] Resumed the update loop");
                else
                {
                    Cycle.StartedAt = DateTime.Now;
                    App.Log.Info("[UpdateCycle] Started the update loop");
                    _started = true;
                }
            }
        }

        /// <summary>
        /// Updates the target UPS (updates per second) and recalculates the update interval.
        /// Propagates the new interval to the <see cref="Procedure"/> and restarts the adjustment timer
        /// to temporarily prevent automatic UPS correction until the new rate stabilizes.
        /// </summary>
        internal void ChangeTargetUPS(uint newTargetUPS)
        {
            _upsMS = 1000.0f / newTargetUPS;
            Procedure.Instance.ProduceCycle = _upsMS;

            App.Log.Info("[UpdateCycle] Target UPS {0} changed to {1} tick's each {2}ms", TargetUPS, newTargetUPS, _upsMS);
            TargetUPS = newTargetUPS;
            AdjustedUPSTimer.Restart();
        }

        /// <summary>
        /// Main update loop responsible for sending <see cref="UpdateTick"/> messages at a consistent interval.
        /// Tracks current UPS, pauses updates when required and halts premature UPS corrections by monitoring
        /// the <see cref="AdjustedUPSTimer"/>. After 2.5 seconds, the timer is stopped, allowing corrections again.
        /// </summary>
        private void UpdateLoop()
        {
            var stopwatch = Stopwatch.StartNew();
            var upsWatch = Stopwatch.StartNew();
            var lastUpdateTime = stopwatch.Elapsed.TotalMilliseconds;

            while (Running)
            {
                var currentTime = stopwatch.Elapsed.TotalMilliseconds;
                var deltaTime = currentTime - lastUpdateTime;

                if (deltaTime >= _upsMS)
                {
                    if (!Paused)
                    {
                        lock (_syncLock)
                            _instance.Tell(new UpdateTick());

                        _updateCount++;
                        lastUpdateTime = currentTime;
                    }
                }

                if (upsWatch.Elapsed.TotalSeconds >= 1.0)
                {
                    CurrentUPS = _updateCount;
                    _updateCount = 0;
                    upsWatch.Restart();
                }

                if (AdjustedUPSTimer.Elapsed.TotalSeconds >= 2.5)
                    AdjustedUPSTimer.Stop();

                Thread.Yield();
            }
        }
    }
}
