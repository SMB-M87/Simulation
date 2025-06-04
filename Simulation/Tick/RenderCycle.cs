using Akka.Actor;
using Akka.Event;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace Simulation.Tick
{
    /// <summary>
    /// Manages the rendering loop of the simulation, maintaining a consistent target frames-per-second (FPS).
    /// The render loop runs in a background thread and periodically sends <see cref="RenderTick"/> messages
    /// to an Akka.NET actor, which is responsible for updating the visual output of the simulation.
    /// </summary>
    internal class RenderCycle
    {
        /// <summary>
        /// Reference to the <see cref="CycleActor"/> that receives <see cref="RenderTick"/> messages to trigger rendering.
        /// </summary>
        private readonly IActorRef _instance;

        /// <summary>
        /// Indicates whether the render loop is currently running. Can be safely accessed by multiple threads.
        /// </summary>
        public volatile bool Running;

        /// <summary>
        /// Frame interval in milliseconds, calculated from the target FPS (e.g., 1000 / FPS).
        /// Determines how often render ticks are sent.
        /// </summary>
        private float _fpsMS;

        /// <summary>
        /// Number of frames rendered in the current one-second interval. Used to calculate FPS.
        /// </summary>
        private uint _frameCount;

        /// <summary>
        /// Gets the target FPS (frames per second) currently configured for the rendering loop.
        /// </summary>
        internal uint TargetFPS { get; private set; }

        /// <summary>
        /// Gets the number of frames that were rendered in the last full second.
        /// </summary>
        internal uint CurrentFPS { get; private set; }

        /// <summary>
        /// Stopwatch used to track when the FPS interval was last manually adjusted via the UI.
        /// Prevents premature correction of the render interval before the system has had time to adapt to the new setting.
        /// </summary>
        internal Stopwatch AdjustedFPSTimer { get; private set; } = new();

        /// <summary>
        /// Synchronization lock used to safely send messages to the actor from the render thread.
        /// </summary>
        private readonly object _syncLock = new();

        /// <summary>
        /// Background thread in which the render loop executes.
        /// </summary>
        private Thread? _renderThread;

        /// <summary>
        /// Initializes a new instance of the <see cref="RenderCycle"/> class.
        /// </summary>
        /// <param name="instance">The actor that receives render tick messages.</param>
        /// <param name="targetFPS">The desired number of frames per second.</param>
        internal RenderCycle(IActorRef instance, uint targetFPS)
        {
            _instance = instance;
            TargetFPS = targetFPS;
            _fpsMS = 1000.0f / targetFPS;
        }

        /// <summary>
        /// Starts the rendering loop on a background thread.
        /// If the loop is already running, this method has no effect.
        /// </summary>
        internal void Start()
        {
            if (Running) return;

            Running = true;
            _renderThread = new Thread(RenderLoop) { IsBackground = true };
            _renderThread.Start();

            App.Log.Info("[RenderCycle] Initialized the render loop");
        }

        /// <summary>
        /// Stops the rendering loop and waits for the render thread to terminate.
        /// </summary>
        internal void Stop()
        {
            Running = false;
            _renderThread?.Join();

            App.Log.Info("[RenderCycle] Destroyed the render loop");
        }

        /// <summary>
        /// Updates the target FPS (frames per second) and recalculates the frame render interval.
        /// Restarts the adjustment timer to temporarily prevent FPS correction until the new rate stabilizes.
        /// </summary>
        /// <param name="newTargetFPS">The new target FPS value.</param>
        internal void ChangeTargetFPS(uint newTargetFPS)
        {
            _fpsMS = 1000.0f / newTargetFPS;
            App.Log.Info("[RenderCycle] Target FPS {0} changed to {1} tick's each {2}ms", TargetFPS, newTargetFPS, _fpsMS);

            TargetFPS = newTargetFPS;
            AdjustedFPSTimer.Restart();
        }

        /// <summary>
        /// Main render loop responsible for sending <see cref="RenderTick"/> messages at a consistent interval.
        /// Tracks and updates current FPS, skips rendering when paused and temporarily disables FPS correction
        /// after manual adjustments by monitoring the <see cref="AdjustedFPSTimer"/>. The timer is stopped after 2.5 seconds.
        /// </summary>
        private void RenderLoop()
        {
            var stopwatch = Stopwatch.StartNew();
            var fpsWatch = Stopwatch.StartNew();
            var lastUpdateTime = stopwatch.Elapsed.TotalMilliseconds;

            while (Running)
            {
                var currentTime = stopwatch.Elapsed.TotalMilliseconds;
                var deltaTime = currentTime - lastUpdateTime;

                if (deltaTime >= _fpsMS)
                {
                    lock (_syncLock)
                        _instance.Tell(new RenderTick());

                    _frameCount++;
                    lastUpdateTime = currentTime;
                }

                if (fpsWatch.Elapsed.TotalSeconds >= 1.0)
                {
                    CurrentFPS = _frameCount;
                    _frameCount = 0;
                    fpsWatch.Restart();
                }

                if (AdjustedFPSTimer.Elapsed.TotalSeconds >= 2.5)
                    AdjustedFPSTimer.Stop();

                Thread.Yield();
            }
        }
    }
}
