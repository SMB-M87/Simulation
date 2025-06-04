using Akka.Actor;
using ITimerScheduler = Akka.Actor.ITimerScheduler;
using IWithTimers = Akka.Actor.IWithTimers;
using ReceiveActor = Akka.Actor.ReceiveActor;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace Simulation.Tick
{
    /// <summary>
    /// Akka.NET actor responsible for coordinating simulation updates and rendering cycles.
    /// 
    /// It handles three types of messages:
    /// <list type="bullet">
    /// <item><description><b><see cref="UpdateTick"/></b>: Triggers simulation updates (UPS).</description></item>
    /// <item><description><b><see cref="RenderTick"/></b>: Triggers visual rendering and display refresh.</description></item>
    /// <item><description><b><see cref="CounterTick"/></b>: Periodically updates cached UPS/FPS values.</description></item>
    /// </list>
    /// 
    /// This actor uses a Windows handle to trigger visual repainting of the simulation window
    /// and relies on Akka's <see cref="ITimerScheduler"/> to periodically send UI counter updates.
    /// </summary>
    internal class CycleActor : ReceiveActor, IWithTimers
    {
        /// <summary>
        /// Tracks the total number of update cycles executed while the simulation is running.
        /// This count increments only when the simulation is not paused and an update cycle occurs.
        /// </summary>
        private ulong _ticks = 0;

        /// <summary>
        /// Counts the number of update ticks in the current second.
        /// </summary>
        private uint _updateCount = 0;

        /// <summary>
        /// Stores the most recent UPS (updates per second) value.
        /// </summary>
        private uint _currentUPS = 0;

        /// <summary>
        /// Stopwatch used to time update cycles and measure UPS.
        /// </summary>
        private readonly Stopwatch _updateStopwatch = new();

        /// <summary>
        /// Counts the number of frames rendered in the current second.
        /// </summary>
        private uint _frameCount = 0;

        /// <summary>
        /// Stores the most recent FPS (frames per second) value.
        /// </summary>
        private uint _currentFPS = 0;

        /// <summary>
        /// Stopwatch used to time render cycles and measure FPS.
        /// </summary>
        private readonly Stopwatch _renderStopwatch = new();

        /// <summary>
        /// Provides access to Akka's timer scheduler for periodic messaging.
        /// </summary>
        public ITimerScheduler Timers { get; set; } = null!;

        /// <summary>
        /// Initializes a new instance of the <see cref="CycleActor"/> class.
        /// </summary>
        /// <param name="handle">The native window handle used to invalidate and refresh the drawing surface.</param>
        public CycleActor()
        {
            _updateStopwatch.Start();
            _renderStopwatch.Start();

            Receive<CounterTick>(_ => UpdateCounters());
            Receive<UpdateTick>(_ => HandleUpdateTick());
            Receive<RenderTick>(_ => HandleRenderTick());
            Receive<FetchUpdateTicks>(_ => HandleFetchUpdateTicks());
            Receive<ResetCycle>(_ => HandleResetCycle());
            Receive<StopCycle>(_ => HandleStopCycle());
        }

        /// <summary>
        /// Called when the actor is started. Begins a periodic timer to send <see cref="CounterTick"/> every ~16ms (~60Hz).
        /// </summary>
        protected override void PreStart()
        {
            Timers.StartPeriodicTimer("counter-timer", new CounterTick(), TimeSpan.FromMilliseconds(1));
            base.PreStart();
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Handles the <see cref="UpdateTick"/> message.</description></item>
        /// <item><description>Checks if <see cref="Cycle.IsRunning"/> is enabled</description></item>
        /// <item><description>If not, calls <see cref="HandleFetchUpdateTicks"/> and exits early.</description></item>
        /// <item><description>If the tick count exceeds <b>TickCap</b>, fetches update ticks and terminates the application.</description></item>
        /// <item><description>If neither condition is met, increments the tick counters and updates the movers and producers.</description></item>
        /// </list>
        /// </summary>
        private void HandleUpdateTick()
        {
            if (!Cycle.IsRunning)
            {
                HandleFetchUpdateTicks();
                Self.Tell(new ResetCycle());
                return;
            }
            else if (Cycle.IsPaused)
            {
                return;
            }

            _ticks++;
            _updateCount++;
            Environment.Instance.Movers.Update();
            Environment.Instance.Producers.Update();

            if (_ticks == Cycle.TickCap)
                Task.Run(() => App.Halt());
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Handles the <see cref="RenderTick"/> message.</description></item>
        /// <item><description>Checks if <see cref="Cycle.IsRendering"/> is enabled</description></item>
        /// <item><description>If not, returns early.</description></item>
        /// <item><description>If rendering is enabled, increments the frame count.</description></item>
        /// <item><description>Renders the units using <b><see cref="Environment.Instance.RenderUnits"/>()</b>.</description></item>
        /// <item><description>If the heatmap setting is enabled, renders the heatmap.</description></item>
        /// <item><description>If display settings are enabled, shows the UPS and FPS metrics.</description></item>
        /// <item><description>Triggers a UI window redraw by calling <b>Interop.InvalidateRect</b> to refresh the display.</description></item>
        /// </list>
        /// </summary>
        private void HandleRenderTick()
        {
            if (!Cycle.IsRendering)
                return;

            _frameCount++;

            Environment.Instance.Render();
            UI.Instance.Refresh();
            Windower.Instance.Refresh();
        }

        /// <summary>
        /// Cancels the active counter timer and caches the current update tick count
        /// in the <see cref="Cycle"/> class for logging or reporting purposes.
        /// </summary>
        private void HandleFetchUpdateTicks()
        {
            Cycle.CachedTicks(_ticks);
        }

        /// <summary>
        /// Handles the reset signal by resetting the update tick count.
        /// </summary>
        private void HandleResetCycle()
        {
            _ticks = 0;
        }

        /// <summary>
        /// Handles the stop signal by shutting down this actor gracefully.
        /// </summary>
        private void HandleStopCycle()
        {
            Timers.Cancel("counter-timer");
            Context.Stop(Self);
        }

        /// <summary>
        /// Periodically updates and caches the UPS and FPS counters once per second.
        /// </summary>
        private void UpdateCounters()
        {
            if (_updateStopwatch.Elapsed.TotalSeconds >= 1.0)
            {
                _currentUPS = _updateCount;
                _updateCount = 0;
                _updateStopwatch.Restart();
                Cycle.CachedUPS(_currentUPS);
            }

            if (_renderStopwatch.Elapsed.TotalSeconds >= 1.0)
            {
                _currentFPS = _frameCount;
                _frameCount = 0;
                _renderStopwatch.Restart();
                Cycle.CachedFPS(_currentFPS);
            }
        }

        /// <summary>
        /// Called when the actor is stopped. Cleans up any remaining resources or behavior.
        /// </summary>
        protected override void PostStop()
        {
            base.PostStop();
        }
    }
}
