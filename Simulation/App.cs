using Akka.Actor;
using Akka.Event;
using ActorSystem = Akka.Actor.ActorSystem;
using ConfigurationFactory = Akka.Configuration.ConfigurationFactory;
using Dumper = Simulation.Util.Dumper;
using Env = System.Environment;
using MQTTClient = Simulation.MQTT.Client;
using Procedure = Simulation.Dummy.Procedure;
using ResetChildren = Simulation.Product.ResetChildren;
using ResetLogger = Simulation.Util.ResetLogger;
using ResetSupervisorState = Simulation.Product.ResetSupervisorState;
using Supervisor = Simulation.Product.Supervisor;
using Vector2 = System.Numerics.Vector2;

namespace Simulation
{
    /// <summary>
    /// Entry point and central controller of the simulation application.
    /// <list type="bullet">
    ///   <item><description><b>Initialization:</b> Loads the environment, actor system, bidding, and tick systems.</description></item>
    ///   <item><description><b>User input:</b> Registers event handlers for mouse, keyboard, and window lifecycle events.</description></item>
    ///   <item><description><b>UI and rendering:</b> Sets up the main simulation window and viewport scaling.</description></item>
    ///   <item><description><b>Lifecycle:</b> Manages startup, runtime ticking, and graceful shutdown.</description></item>
    /// </list>
    /// </summary>
    internal sealed class App
    {
        /// <summary>
        /// Global Akka.NET actor system used to coordinate all actor-based components.
        /// <list type="bullet">
        ///   <item><description><b>Concurrency engine:</b> Manages simulation units, messages, and timers asynchronously.</description></item>
        ///   <item><description><b>Startup:</b> Instantiated from HOCON config during initialization.</description></item>
        ///   <item><description><b>Global scope:</b> Shared across systems like ticking, bidding, and environment logic.</description></item>
        /// </list>
        /// </summary>
        public static ActorSystem System = null!;

        /// <summary>Static reference to the central supervisor actor responsible for managing product actors.</summary>
        public static IActorRef ProductSupervisor { get; private set; } = null!;

        /// <summary>
        /// Global logging interface used to emit simulation log messages via a custom Akka.NET logger.
        /// <list type="bullet">
        ///   <item><description><b>Backed by:</b> The <see cref="Simulation.Util.LoggerActor"/> actor, configured as Akka’s system logger.</description></item>
        ///   <item><description><b>Routing:</b> Automatically categorizes and writes logs to per-actor files and a shared log file.</description></item>
        ///   <item><description><b>Diagnostics:</b> Captures simulation events, warnings, and critical information across the entire actor system.</description></item>
        ///   <item><description><b>Accessible globally:</b> Used throughout the simulation via the static <c>App.Log</c> reference.</description></item>
        /// </list>
        /// </summary>
        public static ILoggingAdapter Log { get; private set; } = null!;

        /// <summary>
        /// Initializes the core runtime components of the simulation.
        /// <list type="bullet">
        ///   <item><description><b>Initialize:</b> Sets up core systems including actor system, environment, settings, and ticking.</description></item>
        ///   <item><description><b>Event hooks:</b> Registers input and window event handlers (resize, keyboard, mouse, destroy).</description></item>
        ///   <item><description><b>Run:</b> Starts the tick cycle and enters the window message loop.</description></item>
        /// </list>
        /// </summary>
        private static void Main()
        {
            var dimension = new Vector2(1920.0f, 1080.0f);

            Windower.Initialize();

            Renderer.Initialize(
                dimension, 
                Windower.Instance.GetHandler(), 
                Windower.Instance.GetClientRect());

            Environment.Initialize();

            System = ActorSystem.Create("root", ConfigurationFactory.ParseString(File.ReadAllText("Resource/akka.hocon")));

            ProductSupervisor = System.ActorOf(
                Props.Create(() => new Supervisor()),
                "Products"
            );

            Log = Logging.GetLogger(System, typeof(App));

            MQTTClient.Initialize();

            Procedure.Initialize();

            Cycle.Initialize();

            UI.Initialize();

            Windower.Instance.RegisterEvents();

            Cycle.StartRenderer();

            Windower.Instance.Run();
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Halt</b>: Stops and resets the simulation when tick-cap reached.</description></item>
        /// <item><description>Logs halt notification and stops the simulation cycle.</description></item>
        /// <item><description>Stops MQTT bidding and dummy bidding procedures if enabled.</description></item>
        /// <item><description>Continuously resets movers and producers until no active procedures remain.</description></item>
        /// <item><description>Clears the Direct2D display and writes out dump logs.</description></item>
        /// <item><description>Logs load-screen transition, publishes <see cref="ResetLogger"/>, loads the splash screen, and reinitializes the cycle.</description></item>
        /// </list>
        /// </summary>
        internal static async void Halt()
        {
            Log.Info("[APP] Tick cap reached or back to load screen interaction");
            Cycle.Halt();

            UI.Instance.Remove();
            UI.Instance.Reset();

            if (UI.Instance.SettingPanel.MQTT.Active)
                await MQTTClient.Instance.Stop();

            if (UI.Instance.SettingPanel.Dummy.Active)
                Procedure.Instance.Stop();

            var tries = 0;

            while (Procedure.Instance.Count > 0)
            {
                if (tries > 10000)
                    ProductSupervisor.Tell(new ResetChildren());

                Environment.Instance.Movers.Reset(true);
                Environment.Instance.Producers.Reset(true);

                tries++;
            }

            Renderer.Instance.Clear();
            Dumper.Log();
            Dumper.Clear();
            ProductSupervisor.Tell(new ResetSupervisorState());

            Log.Info("[APP] Loading screen");
            System.EventStream.Publish(new ResetLogger());

            Environment.Instance.RenderBackground();
            Environment.Instance.LoadScreen();
            UI.Instance.Render();

            Cycle.StartedAt = DateTime.Now;
            Cycle.ResetTickFetch();
            Cycle.StartRenderer();
        }

        /// <summary>
        /// Gracefully shuts down the simulation and cleans up resources.
        /// <list type="bullet">
        ///   <item><description><b>Stop tick cycle:</b> Halts the simulation update loop managed by <see cref="Cycle"/>.</description></item>
        ///   <item><description><b>Stop bidding:</b> If MQTT is enabled, disconnects the client via <see cref="MQTTClient"/>; otherwise stops the dummy bidding <see cref="Procedure"/>.</description></item>
        ///   <item><description><b>Reset state:</b> Clears mover and producer states to ensure a clean termination.</description></item>
        ///   <item><description><b>Release resources:</b> Clears any active draw commands from the renderer.</description></item>
        ///   <item><description><b>Dump state:</b> Persists the simulation state using the <see cref="Dumper"/> utility before exit.</description></item>
        ///   <item><description><b>Exit:</b> Calls <see cref="Env.Exit(int)"/> to terminate the process with exit code 0.</description></item>
        /// </list>
        /// </summary>
        internal static async void Quit()
        {
            Log.Info("[APP] === SHUTDOWN PROCEDURE ACTIVATED ===");
            UI.Instance.Remove();
            Cycle.Stop();

            if (UI.Instance.SettingPanel.MQTT.Active)
                await MQTTClient.Instance.Stop();

            if (UI.Instance.SettingPanel.Dummy.Active)
                Procedure.Instance.Stop();

            var tries = 0;

            while (Procedure.Instance.Count > 0)
            {
                if (tries > 10000)
                    ProductSupervisor.Tell(new ResetChildren());

                Environment.Instance.Movers.Reset(true);
                Environment.Instance.Producers.Reset(true);

                tries++;
            }

            Dumper.Log();

            Log.Info("[APP] === THE END ===");
            Renderer.Instance.Clear();
            Renderer.Instance.Dispose();
            Env.Exit(0);
        }
    }
}
