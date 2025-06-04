using Akka.Actor;
using Akka.Event;
using ActorRefs = Akka.Actor.ActorRefs;
using CycleActor = Simulation.Tick.CycleActor;
using FetchUpdateTicks = Simulation.Tick.FetchUpdateTicks;
using IActorRef = Akka.Actor.IActorRef;
using Props = Akka.Actor.Props;
using RenderCycle = Simulation.Tick.RenderCycle;
using ResetCycle = Simulation.Tick.ResetCycle;
using StopCycle = Simulation.Tick.StopCycle;
using UpdateCycle = Simulation.Tick.UpdateCycle;

namespace Simulation
{
    /// <summary>
    /// Provides centralized control over the simulation's update and render cycles,
    /// coordinating timing, pausing and performance metrics.
    /// 
    /// Internally manages separate background loops for UPS (update-per-second) and FPS (frame-per-second),
    /// which send messages to the <see cref="CycleActor"/> for simulation logic and rendering.
    /// </summary>
    internal static class Cycle
    {
        /// <summary>
        /// The date and time when the simulation was initialized via <see cref="Initialize"/> and eventually unpaused for the first time.
        /// </summary>
        internal static DateTime StartedAt { get; set; }

        /// <summary>
        /// The date and time when the simulation was stopped via <see cref="Stop"/>.
        /// </summary>
        internal static DateTime StoppedAt { get; private set; }

        /// <summary>
        /// Gets the total duration for which the simulation has been paused since the last start.
        /// Returns <see cref="TimeSpan.Zero"/> if the update manager or pause timer is not initialized.
        /// </summary>
        internal static TimeSpan PausedDuration => _updateManager?.PauseTimer.Elapsed ?? TimeSpan.Zero;

        /// <summary>
        /// Indicates whether the update loop is currently running.
        /// </summary>
        internal static bool IsRunning => _updateManager?.Running ?? false;

        /// <summary>
        /// Indicates whether the render loop is currently running.
        /// </summary>
        internal static bool IsRendering => _renderManager?.Running ?? false;

        /// <summary>
        /// Indicates whether the update loop is currently paused.
        /// </summary>
        internal static bool IsPaused => _updateManager?.Paused ?? false;

        /// <summary>
        /// The maximum number of update ticks the simulation should run before stopping.
        /// Used for benchmarking consistent tick counts across runs.
        /// Defaults to <see cref="ulong.MaxValue"/> when uncapped to prevent overflow.
        /// </summary>
        internal static ulong TickCap { get; set; }

        /// <summary>
        /// The total number of update ticks since the system started.
        /// </summary>
        internal static ulong UpdateTicks { get; private set; }

        /// <summary>
        /// Manages the render cycle, sending render ticks at the configured FPS (frames per second).
        /// </summary>
        private static RenderCycle? _renderManager;

        /// <summary>
        /// Gets the FPS value measured by the actor (cached every second).
        /// </summary>
        internal static uint ActorFPS => _cachedFPS;

        /// <summary>
        /// Gets the actual FPS measured from the render loop.
        /// </summary>
        internal static uint CycleFPS => _renderManager?.CurrentFPS ?? 0;

        /// <summary>
        /// Gets the currently configured target frames per second (FPS).
        /// </summary>
        internal static uint TargetFPS => _renderManager?.TargetFPS ?? 0;

        /// <summary>
        /// Manages the update cycle, sending update ticks at the configured UPS (updates per second).
        /// </summary>
        private static UpdateCycle? _updateManager;

        /// <summary>
        /// Gets the UPS value measured by the actor (cached every second).
        /// </summary>
        internal static uint ActorUPS => _cachedUPS;

        /// <summary>
        /// Gets the actual UPS measured from the update loop.
        /// </summary>
        internal static uint CycleUPS => _updateManager?.CurrentUPS ?? 0;

        /// <summary>
        /// Gets the currently configured target updates per second (UPS).
        /// </summary>
        internal static uint TargetUPS => _updateManager?.TargetUPS ?? 0;

        /// <summary>
        /// Updates the cached FPS value from the actor, used for display or diagnostics.
        /// </summary>
        /// <param name="FPS">The most recent FPS value measured by the actor.</param>
        internal static void CachedFPS(uint FPS)
        {
            lock (_lock)
            {
                _cachedFPS = FPS;

                if (IsPaused || ActorFPS < 10 || _renderManager!.AdjustedFPSTimer.IsRunning)
                    return;

                var margin = TargetFPS * 0.05f;

                if (_cachedFPS < TargetFPS - margin)
                {
                    if (_cachedFPSoffset > 3)
                    {
                        _cachedFPSoffset = 0;
                        ChangeRenderInterval(_cachedFPS);
                        UI.Instance.SettingPanel.FPS.Slider.Interval = _cachedFPS;

                        if (UI.Instance.SettingPanel.Visible)
                            UI.Instance.SettingPanel.FPS.Slider.Render();

                        _renderManager?.AdjustedFPSTimer.Restart();
                    }
                    else
                    {
                        _cachedFPSoffset++;
                        _renderManager!.AdjustedFPSTimer.Restart();
                    }
                }
                else
                    _cachedFPSoffset = 0;
            }
        }
        private static uint _cachedFPS;
        private static uint _cachedFPSoffset;

        /// <summary>
        /// Updates the cached UPS value from the actor, used for display or diagnostics.
        /// </summary>
        /// <param name="UPS">The most recent UPS value measured by the actor.</param>
        internal static void CachedUPS(uint UPS)
        {
            lock (_lock)
            {
                _cachedUPS = UPS;

                if (IsPaused || ActorUPS < 10 || _updateManager!.AdjustedUPSTimer.IsRunning)
                    return;

                var margin = TargetUPS * 0.05f;

                if (_cachedUPS < TargetUPS - margin)
                {
                    _cachedUPSoffset++;

                    if (_cachedUPSoffset > 3)
                    {
                        _cachedUPSoffset = 0;

                        ChangeUpdateInterval(_cachedUPS);
                        UI.Instance.SettingPanel.UPS.Slider.Interval = _cachedUPS;

                        if (UI.Instance.SettingPanel.Visible)
                            UI.Instance.SettingPanel.UPS.Slider.Render();

                        _updateManager?.AdjustedUPSTimer.Restart();
                    }
                    else
                    {
                        _cachedUPSoffset++;
                        _updateManager!.AdjustedUPSTimer.Restart();
                    }
                }
                else
                    _cachedUPSoffset = 0;
            }
        }
        private static uint _cachedUPS;
        private static uint _cachedUPSoffset;

        /// <summary>
        /// Provides the total number of update ticks since the simulation started.
        /// </summary>
        internal static void CachedTicks(ulong ticks)
        {
            lock (_lock)
            {
                UpdateTicks = ticks;
                _ticksDelivered = true;
            }
        }
        private static bool _ticksDelivered = false;

        /// <summary>
        /// Reference to the instance of the <see cref="CycleActor"/> responsible for handling update and render ticks.
        /// </summary>
        private static IActorRef _instance = ActorRefs.Nobody;

        /// <summary>
        /// Lock object to ensure thread-safe initialization of the update and render managers.
        /// </summary>
        private static readonly object _lock = new();

        /// <summary>
        /// Initializes and starts the update and render cycles by creating the <see cref="CycleActor"/> 
        /// and configuring the update and render managers with the specified target UPS and FPS values. 
        /// Also records the start time of the simulation for time tracking purposes.
        /// Ensures thread safety during initialization using a lock.
        /// </summary>
        /// <param name="targetUPS">Target number of updates per second.</param>
        /// <param name="targetFPS">Target number of frames per second.</param>
        /// <param name="tickCap">Target number of simulated tick's.</param>
        internal static void Initialize(
            uint targetUPS = 200,
            uint targetFPS = 60,
            ulong tickCap = ulong.MaxValue
            )
        {
            lock (_lock)
            {
                App.Log.Info("[Cycle] Start initialize");
                _instance = App.System.ActorOf(Props.Create(() => new CycleActor()), "CycleActor");

                App.Log.Info("[Cycle] Initialized UpdateCycle(instance: {0}, targetUPS: {1})", _instance.Path, targetUPS);
                _updateManager = new UpdateCycle(_instance, targetUPS);

                App.Log.Info("[Cycle] Initialized RenderCycle(instance: {0}, targetFPS: {1})", _instance.Path, targetFPS);
                _renderManager = new RenderCycle(_instance, targetFPS);

                TickCap = tickCap > 0 ? tickCap : ulong.MaxValue;
                StartedAt = DateTime.Now;
                App.Log.Info("[Cycle] Finished initializing");
            }
        }

        /// <summary>
        /// Starts the render loop.
        /// </summary>
        internal static void StartRenderer()
        {
            App.Log.Info("[Cycle] Notify to start the render loop");
            _renderManager?.Start();
        }

        /// <summary>
        /// Starts the update loop.
        /// </summary>
        internal static void Start()
        {
            App.Log.Info("[Cycle] Notify to start the update loop");
            _updateManager?.Start();
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Halt</b>: Stops and resets the simulation at tick-cap reached.</description></item>
        /// <item><description>Logs halt notification and stops the simulation cycle.</description></item>
        /// <item><description>Stops MQTT bidding and dummy bidding procedures if enabled.</description></item>
        /// <item><description>Continuously resets movers and producers until no active procedures remain.</description></item>
        /// <item><description>Clears the Direct2D display and writes out dump logs.</description></item>
        /// <item><description>Logs load-screen transition, publishes <see cref="ResetLogger"/>, loads the splash screen, and reinitializes the cycle.</description></item>
        /// </list>
        /// </summary>
        internal static void Halt()
        {
            App.Log.Info("[Cycle] Stops the render and update processes");

            if (!IsPaused)
                _updateManager?.TogglePause();

            _renderManager?.Stop();
            _updateManager?.Stop();

            while (!_ticksDelivered)
                _instance.Tell(new FetchUpdateTicks());

            StoppedAt = DateTime.Now;
            _updateManager!.PauseTimer.Start();

            App.Log.Info("[CycleActor] Delivered the total update tick count: {0}", UpdateTicks);
            _instance.Tell(new ResetCycle());
        }

        /// <summary>
        /// Gracefully stops the simulation stopping the update and render managers,
        /// waiting for UPS and FPS activity to cease, then fetching the final update tick count.
        /// After the tick count is captured, sends a <see cref="StopCycle"/> message to terminate the cycle actor.
        /// </summary>
        internal static void Stop()
        {
            App.Log.Info("[Cycle] Stops the render and update processes");
            _renderManager?.Stop();
            _updateManager?.Stop();

            while (!_ticksDelivered)
                _instance.Tell(new FetchUpdateTicks());

            StoppedAt = DateTime.Now;
            _updateManager!.PauseTimer.Start();
            App.Log.Info("[CycleActor] Delivered the total update tick count: {0}", UpdateTicks);
            _instance.Tell(new StopCycle());
        }

        internal static void ResetTickFetch()
        {
            _ticksDelivered = false;
            UpdateTicks = 0;
        }

        /// <summary>
        /// Toggles the paused state of the update loop.
        /// </summary>
        internal static void Toggle()
        {
            _cachedFPSoffset = 0;
            _cachedUPSoffset = 0;
            _updateManager?.TogglePause();
        }

        /// <summary>
        /// Dynamically changes the render interval (FPS) of the simulation loop.
        /// </summary>
        /// <param name="newInterval">The new frames-per-second value.</param>
        internal static void ChangeRenderInterval(uint newInterval)
        {
            _cachedFPSoffset = 0;
            _renderManager?.ChangeTargetFPS(newInterval);
        }

        internal static string RenderText()
        {
            var margin = CycleFPS * 0.05f;
            string text;

            if (ActorFPS < CycleFPS - margin || ActorFPS > CycleFPS + margin)
            {
                text = $"{ActorFPS}:{CycleFPS}";
            }
            else
                text = $"{ActorFPS}";

            return $"{text} / {TargetFPS} FPS";
        }

        /// <summary>
        /// Dynamically changes the update interval (UPS) of the simulation loop.
        /// </summary>
        /// <param name="newInterval">The new updates-per-second value.</param>
        internal static void ChangeUpdateInterval(uint newInterval)
        {
            _cachedUPSoffset = 0;
            _updateManager?.ChangeTargetUPS(newInterval);
        }

        internal static string UpdateText()
        {
            var margin = CycleUPS * 0.05f;
            string text;

            if (ActorUPS < CycleUPS - margin || ActorUPS > CycleUPS + margin)
            {
                text = $"{ActorUPS}:{CycleUPS}";
            }
            else
                text = $"{ActorUPS}";

            return $"{text} / {TargetUPS} UPS";
        }
    }
}
