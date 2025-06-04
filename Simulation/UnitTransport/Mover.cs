using Akka.Actor;
using Simulation.UnitTransport.Cost;
using Simulation.UnitTransport.NavComposite;
using Color = Simulation.Util.Color;
using IOPath = System.IO.Path;
using Model = Simulation.Unit.Model;
using MQTTClient = Simulation.MQTT.Client;
using Producer = Simulation.UnitProduction.Producer;
using State = Simulation.Unit.State;
using TextStyles = Simulation.Util.TextStyles;
using TransportCompleted = Simulation.Dummy.TransportCompleted;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Represents a transport unit ("Mover") in the simulation.
    /// A mover is a physics-based agent responsible for transporting products from one location to another.
    /// It interacts with its environment, renders itself visually, and optionally logs its state.
    /// <list type="bullet">
    ///   <item><description><b>Transport:</b> Carries products to producer destinations, confirms arrivals, and tracks execution time.</description></item>
    ///   <item><description><b>Environment interaction:</b> Navigates around other agents and borders using steering and pathfinding logic.</description></item>
    ///   <item><description><b>Simulation tracking:</b> Maintains statistics such as idle time, movement distance, and execution counters.</description></item>
    ///   <item><description><b>Debug rendering:</b> Supports visual overlays for path, velocity, collision radius, etc., through Direct2D.</description></item>
    ///   <item><description><b>Logging:</b> If enabled, writes detailed transport and movement logs to per-agent text files.</description></item>
    /// </list>
    /// </summary>
    /// <param name="id">The numeric ID used to uniquely identify this mover within its model group.</param>
    /// <param name="model">The model type of the mover (e.g., APM4220).</param>
    /// <param name="position">The initial position of the mover in world coordinates.</param>
    internal sealed class Mover(
        int id,
        Model model,
        Vector2 position
        ) : MovableBody(
            $"{model}_{id}",
            model,
            position,
            MoverModel.Specs[model].Dimension
            )
    {
        /// <summary>
        /// Transport cost calculator used to determine the cost of reaching a target position.
        /// <list type="bullet">
        ///   <item><description><b>Cost Calculation:</b> Calculates the distance and time cost associated with moving the mover to the target.</description></item>
        /// </list>
        /// </summary>
        internal TransportCost Cost { get; private set; } = null!;

        /// <summary>
        /// Navigation strategy used for steering and path planning.
        /// <list type="bullet">
        ///   <item><description><b>Steering and Pathfinding:</b> Handles the movement of the mover based on its environment.</description></item>
        /// </list>
        /// </summary>
        internal Navigation Navigation { get; private set; } = null!;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>True while transport is active:</b> This flag is set to true when a transport operation is in progress.</description></item>
        ///   <item><description><b>Activates tracking and guidance:</b> Used to enable tracking of transport ticks and guidance logic for pathfinding and steering.</description></item>
        /// </list>
        /// </summary>
        internal bool Active { get; private set; } = false;

        /// <summary>
        /// Represents the state that has been set via user interaction by clicking on the mover.
        /// </summary>
        internal bool Disabled { get; set; } = false;

        /// <summary>
        /// Indicates whether a delivery confirmation has been sent for the current product.
        /// Prevents duplicate <c>TransportCompleted</c> messages.
        /// <list type="bullet">
        ///   <item><description><b>Prevents duplication:</b> Ensures that only one transport completion message is sent for each product.</description></item>
        /// </list>
        /// </summary>
        internal bool Completed { get; set; } = true;

        /// <summary>
        /// Number of completed transport operations.
        /// <list type="bullet">
        ///   <item><description><b>Transport Counter:</b> Tracks how many transport operations have been completed by this mover.</description></item>
        /// </list>
        /// </summary>
        internal ulong Count { get; private set; } = 0;

        /// <summary>
        /// Total number of ticks spent unassigned and thus idle.
        /// <list type="bullet">
        ///   <item><description><b>Idle Time:</b> Tracks the total number of simulation ticks the mover has been unassigned.</description></item>
        /// </list>
        /// </summary>
        internal ulong Idle { get; private set; } = 0;

        /// <summary>
        /// Cumulative distance traveled by this mover, in millimeters.
        /// <list type="bullet">
        ///   <item><description><b>Distance Tracking:</b> Tracks the total distance traveled by the mover during simulation.</description></item>
        /// </list>
        /// </summary>
        internal ulong Distance { get; private set; } = 0;

        /// <summary>
        /// Fractional distance below 1mm accumulated during simulation steps.
        /// Used for precise movement tracking.
        /// <list type="bullet">
        ///   <item><description><b>Precision:</b> Tracks fine-grained movement to ensure high accuracy in position updates.</description></item>
        /// </list>
        /// </summary>
        internal double FractionalDistance { get; private set; } = 0.0f;

        /// <summary>
        /// Number of ticks this mover has been stationary while being allocated to a product.
        /// <list type="bullet">
        ///   <item><description><b>Motionless Tracking:</b> Tracks the number of ticks where the mover remains still during transport.</description></item>
        /// </list>
        /// </summary>
        internal ulong ActiveStationary { get; private set; } = 0;

        /// <summary>
        /// Number of idle ticks where the mover has been stationary while being idle.
        /// <list type="bullet">
        ///   <item><description><b>Idle and Stationary:</b> Tracks when the mover is idle and stationary, awaiting tasks or assignment.</description></item>
        /// </list>
        /// </summary>
        internal ulong IdleStationary { get; private set; } = 0;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Transport ticks tracking:</b> Tracks the number of ticks the mover has been transporting the current product.</description></item>
        ///   <item><description><b>Reset upon transport completion:</b> This counter is reset when the transport operation is completed via <see cref="InteractionCompleted"/>.</description></item>
        /// </list>
        /// </summary>
        internal ulong Transport { get; private set; } = 0;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Distance traveled during transport:</b> Tracks the total distance (in millimeters) the mover has traveled for the current transport operation.</description></item>
        ///   <item><description><b>Reset upon transport completion:</b> This value is reset when the transport is completed via <see cref="InteractionCompleted"/>.</description></item>
        /// </list>
        /// </summary>
        internal float TransportDistance { get; private set; } = 0;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description>The countdown is used to delay the reassignment of the mover parking space after completing a transport to prevent immediate reassignment.</description></item>
        /// </list>
        /// </summary>
        internal uint _parking = 0;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description>The default countdown value used to delay the reassignment of the mover parking space.</description></item>
        /// </list>
        /// </summary>
        private readonly uint _defaultParkCountdown = 10;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Collision area radius:</b> The radius used for rendering the mover's primary collision area, for visual and collision detection purposes.</description></item>
        /// </list>
        /// </summary>
        private float _renderRadius;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Detection area radius:</b> The radius used for rendering the mover's detection/sensing area, determining how far the mover can detect nearby objects or other agents.</description></item>
        /// </list>
        /// </summary>
        private float _renderDetectionRadius;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Rendered dimensions:</b> The screen-space dimensions of the mover’s body, taking into account the current zoom scale.</description></item>
        /// </list>
        /// </summary>
        private Vector2 _renderDimension;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Debug visualization mask:</b> A bitmask that controls the optional debug overlays, such as path, vectors, and collision radii for the mover.</description></item>
        /// </list>
        /// </summary>
        private DebugFlag _debug = 0;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Debug detection multiplier:</b> A scaling factor used to adjust the size of the debug detection radius based on the mover's maximum force.</description></item>
        /// </list>
        /// </summary>
        private readonly int _debugDetection = 8;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Waypoints in path visualization:</b> The maximum number of waypoints that will be shown in the visualized path during debugging.</description></item>
        /// </list>
        /// </summary>
        private readonly uint _pathLookAHead = 15;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Lazily-initialized motion logger:</b> A thread-safe log stream that is initialized when first accessed. Used to record the mover's motion data.</description></item>
        ///   <item><description><b>Thread-safe access:</b> Ensures that only one thread can access and modify the log file at a time using a lock mechanism.</description></item>
        /// </list>
        /// </summary>
        private StreamWriter Logger
        {
            get
            {
                lock (_logLock)
                {
                    if (_log == null)
                        SetupLog();
                    return _log!;
                }
            }
        }

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Backing field for Logger:</b> Stores the instance of the log file writer. Initialized only when first accessed via <see cref="Logger"/>.</description></item>
        ///   <item><description><b>Null on first use:</b> The <see cref="_log"/> field is set to null initially, and it's lazily initialized when the Logger property is accessed.</description></item>
        /// </list>
        /// </summary>
        private StreamWriter? _log = null;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Lock object for synchronization:</b> A dedicated lock used to synchronize access to the <see cref="Logger"/>. Ensures thread safety when writing to the log file.</description></item>
        ///   <item><description><b>Prevents race conditions:</b> The lock ensures that only one thread can access the log file at a time, preventing potential data corruption or inconsistency.</description></item>
        /// </list>
        /// </summary>
        private readonly object _logLock = new();

        /// <summary>
        /// Format string used for logging motion data entries in the mover log file.
        /// The format includes timestamped position and velocity information:
        /// <list type="bullet">
        ///   <item><description><b>[0]:</b> Timestamp formatted as yyyy-MM-dd HH:mm:ss.ffff.</description></item>
        ///   <item><description><b>[1]:</b> Previous X position.</description></item>
        ///   <item><description><b>[2]:</b> Previous Y position.</description></item>
        ///   <item><description><b>[3]:</b> Velocity X component.</description></item>
        ///   <item><description><b>[4]:</b> Velocity Y component.</description></item>
        ///   <item><description><b>[5]:</b> Current X position.</description></item>
        ///   <item><description><b>[6]:</b> Current Y position.</description></item>
        /// </list>
        /// </summary>
        private const string _motionFormat = "[{0,-24}]  {1,18}  {2,-16}  +  {3,18}  {4,-16}  =>  {5,15}  {6,-15}";

        internal void Setup(Dictionary<(int, int), uint> grid)
        {
            Cost = new EucledianDistance();
            Navigation = new(new BasicSteering(), new AStarHeatmap(grid));
        }

        /// <summary>
        /// Initializes the mover’s log file and writes static metadata and headers:
        /// <list type="bullet">
        ///   <item><description><b>Path setup:</b> Creates a per-mover file in the output directory under <c>Movers/{ID}.txt</c>.</description></item>
        ///   <item><description><b>Metadata logging:</b> Logs initial configuration such as navigation model, max speed, and max force.</description></item>
        ///   <item><description><b>Log format header:</b> Writes a labeled header row for real-time position update entries.</description></item>
        /// </list>
        /// </summary>
        internal void SetupLog()
        {
            var path = IOPath.Combine(
                AppContext.BaseDirectory,
                $"{Environment.Instance.OutputDir}",
                $"Movers/{ID}.txt"
                );
            Directory.CreateDirectory(IOPath.GetDirectoryName(path)!);
            _log = new StreamWriter(path, append: true) { AutoFlush = true };

            _log.WriteLine($"Navigation: {Navigation.GetID()}");
            _log.WriteLine($"Max speed: {MaxSpeed} mm/tick");
            _log.WriteLine($"Max force: {MaxForce} mm/tick\n");
            _log.WriteLine(
                string.Format(_motionFormat, "Datetime Position Update",
                "Old Position X", "Y",
                "Velocity X", "Y",
                "Position X", "Y")
                );
        }

        /// <summary>
        /// Computes the movement cost to a given destination:
        /// <list type="bullet">
        ///   <item><description><b>Checks product status:</b> If the mover is carrying a product and not explicitly allowed (<c>allocated</c> is false), the cost is <c>ulong.MaxValue</c>.</description></item>
        ///   <item><description><b>Delegates cost computation:</b> Otherwise, uses the configured cost model (e.g., Manhattan, Euclidean) to calculate the distance from current position to the destination.</description></item>
        /// </list>
        /// </summary>
        /// <param name="destination">The target destination to compute cost to.</param>
        /// <param name="allocated">Optional flag to override product constraint when true.</param>
        /// <returns>Movement cost as an unsigned integer. Returns <c>ulong.MaxValue</c> if invalid.</returns>
        internal ulong GetCost(Vector2 destination, bool allocated = false)
        {
            if (!allocated && ServiceRequester != ActorRefs.Nobody)
                return ulong.MaxValue;

            return Cost.Calculate(Center, destination);
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Allocate</b>: Assigns the specified actor reference to the <b>Product</b> field, marking it as in use.</description></item>
        /// </list>
        /// </summary>
        internal void Allocate(IActorRef actor)
        {
            ServiceRequester = actor;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                 "[{0,-24}] {1,-38} {2,-50}\n",
                 DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                 $"ALLOCATED TO [{ServiceRequester.Path.Name}]"
                ));
            }
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Deallocate</b>: Resets the <b>Product</b> field to <b>ActorRefs.Nobody</b>, marking it as unassigned.</description></item>
        /// </list>
        /// </summary>
        internal void Deallocate()
        {
            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                 "[{0,-24}] {1,-38} {2,-50}\n",
                 DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                 $"DEALLOCATED FROM [{ServiceRequester.Path.Name}]"
                ));
            }
            ServiceRequester = ActorRefs.Nobody;
        }

        /// <summary>
        /// Initiates a new transport operation directed at the specified producer:
        /// <list type="bullet">
        ///   <item><description><b>Assigns product:</b> Sets the mover’s <c>Product</c> reference to the provided actor.</description></item>
        ///   <item><description><b>Clears confirmation flag:</b> Resets <c>InformConfimation</c> to ensure delivery notification is sent later.</description></item>
        ///   <item><description><b>Sets destination:</b> Uses the target producer’s processing center as the goal position.</description></item>
        ///   <item><description><b>Registers mover:</b> Adds the mover to the producer’s processing queue.</description></item>
        ///   <item><description><b>Starts tracking:</b> Enables internal state flags to track transport progress and trigger updates.</description></item>
        ///   <item><description><b>Logs event:</b> If logging is enabled, records a formatted <c>START TRANSPORT TO</c> message with timestamp and coordinates.</description></item>
        /// </list>
        /// </summary>
        /// <param name="producer">The destination producer that will receive the product.</param>
        internal void StartTransport(Producer producer)
        {
            Completed = false;
            Destination = producer.Processer.Center;
            Reset = true;
            Transport = 0;
            DestinationUnreachable = false;
            SwapDestination = Vector2.Zero;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                 "[{0,-24}] {1,-38} {2,-50}",
                 DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                 " - - - - - - - - - - - - ",
                 $"START TRANSPORT TO {producer.ID} ({producer.Processer.Center.X}  {producer.Processer.Center.Y}) FOR [{ServiceRequester.Path.Name}]"
                ));
            }

            if (Environment.Instance.Parkings.LeaveSpace(this) && UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                "[{0,-24}] {1,-38} {2,-35}",
                DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                "REMOVED PARKING SPACE ALLOCATION"
                ));
            }
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>InteractionCompleted</b>: Marks the end of a mover's interaction with a producer.</description></item>
        /// <item><description>Resets the destination, transport counter, and distance.</description></item>
        /// <item><description>Increments the executed transport count and sets the internal completion countdown.</description></item>
        /// <item><description>If mover logging is enabled, logs a timestamped message identifying the completed interaction.</description></item>
        /// </list>
        /// </summary>
        internal void InteractionCompleted(uint ticks)
        {
            Destination = Vector2.Zero;
            Count++;
            _parking = 4;
            TransportDistance = 0;
            ActiveStationary = ActiveStationary > ticks ? ActiveStationary - ticks : 0;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                 "[{0,-24}] {1,-38} {2,-50}\n",
                 DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                 $"INTERACTION ON [{ServiceRequester.Path.Name}] COMPLETED after {ticks} tick's"
                ));
            }

            Destination = Environment.Instance.GetLeastCrowdedNearbyPosition(
                Center,
                Dimension,
                CellWeight,
                2
                );
            Active = true;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                "[{0,-24}] {1,-38} {2,-35}",
                DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                $"MOVING OUT PROCESSING RANGE TO {Destination.X}  {Destination.Y}"
                ));
            }
        }

        /// <summary>
        /// Handles the case when a mover’s transport interaction or producer interaction is suspended (bailed) before completion.
        /// <list type="bullet">
        ///   <item><description>Clears the current destination and resets parking countdown and transport distance.</description></item>
        ///   <item><description>Logs a “suspended” event if mover logging is enabled.</description></item>
        ///   <item><description>Computes a new fallback destination via <see cref="Environment.GetLeastCrowdedNearbyPosition"/>.</description></item>
        ///   <item><description>Re-enables movement by setting <c>_allowedToMove</c> to true.</description></item>
        ///   <item><description>Logs the new “moving out” destination if mover logging is enabled.</description></item>
        /// </list>
        /// </summary>
        internal void InteractionBailed()
        {
            Destination = Vector2.Zero;
            _parking = 4;
            TransportDistance = 0;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                 "[{0,-24}] {1,-38} {2,-50}\n",
                 DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                 $"INTERACTION ON [{ServiceRequester.Path.Name}] WAS SUSPENDED"
                ));
            }

            Destination = Environment.Instance.GetLeastCrowdedNearbyPosition(
                Center,
                Dimension,
                CellWeight,
                2
                );
            Active = true;

            if (UI.Instance.SettingPanel.LogMovers.Active)
            {
                Logger!.WriteLine(string.Format(
                "[{0,-24}] {1,-38} {2,-35}",
                DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                " - - - - - - - - - - - - ",
                $"MOVING OUT PROCESSING RANGE TO {Destination.X}  {Destination.Y}"
                ));
            }
        }

        /// <summary>
        /// Updates the mover’s simulation state for the current tick:
        /// <list type="bullet">
        ///   <item><description><b>Collision countdown:</b> Decrements the countdown if active.</description></item>
        ///   <item><description><b>Start signal:</b> Clears the current path if a transport start signal was received.</description></item>
        ///   <item><description><b>Transport tracking:</b> Increments the transport tick counter while actively transporting.</description></item>
        ///   <item><description><b>Parking logic:</b> Attempts to assign or reallocate parking based on transport state.</description></item>
        ///   <item><description><b>Navigation:</b> Applies guidance logic if the mover is actively transporting.</description></item>
        ///   <item><description><b>Movement handling:</b> If the position has changed, logs the distance traveled and updates the mover’s grid registration.</description></item>
        ///   <item><description><b>Idle tracking:</b> Increments <c>IdleMotionlessCounter</c> if the mover is idle and stationary.</description></item>
        ///   <item><description><b>Motion logging:</b> If enabled, records a movement entry to the log with timestamp and coordinates.</description></item>
        ///   <item><description><b>Arrival check:</b> Evaluates whether the mover has reached its destination and triggers arrival logic if so.</description></item>
        /// </list>
        /// </summary>
        internal void Update()
        {
            if (Disabled)
                return;

            var oldPos = Center;

            if (Collided > 0)
                Collided--;

            if (_parking > 0)
                _parking--;

            if (ServiceRequester == ActorRefs.Nobody)
                Idle++;
            else if (Active)
                Transport++;

            Parking();

            var context = new NavigationContext()
            {
                Agent = this,
                Environment = Environment.Instance,
                Neighbors = Environment.Instance.Movers.GetPossibleCollisions(this),
                Borders = Environment.Instance.Borders.GetPossibleCollisions(this)
            };

            if (Active)
                Navigation.Guidance(context);

            if (Center != oldPos)
            {
                var deltaDistance = (Center - oldPos).Length();
                FractionalDistance += deltaDistance;

                if (FractionalDistance >= 1.0)
                {
                    var whole = (ulong)FractionalDistance;
                    Distance += whole;
                    FractionalDistance -= whole;
                }

                if (ServiceRequester != ActorRefs.Nobody && Active)
                    TransportDistance += deltaDistance;

                Environment.Instance.Movers.UpdateMoverPosition(
                    ID,
                    oldPos,
                    Center,
                    Dimension,
                    CellWeight
                    );

                if (UI.Instance.SettingPanel.LogMovers.Active)
                    Logger.WriteLine(string.Format(
                         _motionFormat,
                         DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                         $"{oldPos.X}",
                         $"{oldPos.Y}",
                         $"{Velocity.X}",
                         $"{Velocity.Y}",
                         $"{Center.X}",
                         $"{Center.Y}"
                    ));
            }
            else if (ServiceRequester != ActorRefs.Nobody)
                ActiveStationary++;
            else
                IdleStationary++;

            Arrived();

            if (Reset)
            {
                Path = [];
                Active = true;
                Reset = false;
            }
        }

        /// <summary>
        /// Executes the mover’s parking behavior based on its current state and context.
        /// <list type="bullet">
        ///   <item><description><b>Assigns space:</b> If idle and without a destination, assigns a parking space after a cooldown period.</description></item>
        ///   <item><description><b>Relocates:</b> If near its current destination, checks for and relocates to a better neighboring parking space.</description></item>
        ///   <item><description><b>Blocked state:</b> Evaluates collision context to determine if the mover should toggle between <c>Alive</c> and <c>Blocked</c> states.</description></item>
        ///   <item><description><b>Release space:</b> If carrying a product, releases its assigned parking space to allow reuse.</description></item>
        ///   <item><description><b>Logging:</b> Logs parking actions and state transitions if mover logging is enabled.</description></item>
        /// </list>
        /// </summary>
        private void Parking()
        {
            if (_parking > 0 || ServiceRequester != ActorRefs.Nobody)
                return;

            if (Destination == Vector2.Zero || !Environment.Instance.Parkings.IsParkingSpace(Destination))
            {
                if (Environment.Instance.Parkings.AssignSpace(this))
                {
                    Reset = true;

                    if (UI.Instance.SettingPanel.LogMovers.Active)
                        Logger!.WriteLine(string.Format(
                         "[{0,-24}] {1,-38} {2,-50}",
                         DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                         " - - - - - - - - - - - - ",
                         $"START PARKING ({Destination.X}  {Destination.Y})"
                        ));
                }
            }
            else if ((Destination - Center).Length() < Dimension.Length() * 2 && Environment.Instance.Parkings.CheckNeighbor(this))
            {
                Active = true;

                if (UI.Instance.SettingPanel.LogMovers.Active)
                    Logger!.WriteLine(string.Format(
                        "[{0,-24}] {1,-38} {2,-50}",
                        DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                        " - - - - - - - - - - - - ",
                        $"CHANGED PARKING ALLOCATION TO ({Destination.X}  {Destination.Y})"
                    ));
            }

            if (!Disabled && IsBlocked(Environment.Instance.Movers.GetPossibleCollisions(this),
                          Environment.Instance.Borders.GetPossibleCollisions(this)))
            {
                if (State == State.Alive)
                {
                    ToggleState();

                    if (UI.Instance.SettingPanel.LogMovers.Active)
                        Logger!.WriteLine(string.Format(
                        "[{0,-24}] {1,-38} {2,-35}",
                        DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                        " - - - - - - - - - - - - ",
                        "STATE BECAME BLOCKED"
                        ));
                }
            }
            else if (!Disabled && State == State.Blocked)
            {
                ToggleState();

                if (UI.Instance.SettingPanel.LogMovers.Active)
                    Logger!.WriteLine(string.Format(
                     "[{0,-24}] {1,-38} {2,-35}",
                     DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                     " - - - - - - - - - - - - ",
                     "STATE BECAME ALIVE"
                    ));
            }
            _parking = _defaultParkCountdown;
        }

        /// <summary>
        /// Checks if the mover has arrived at its destination within a small tolerance margin.
        /// <list type="bullet">
        ///   <item><description><b>Transport confirmation:</b> If carrying a product and confirmation hasn't been sent, sends a <c>TransportCompleted</c> message (via MQTT or Akka).</description></item>
        ///   <item><description><b>Parking optimization:</b> If idle (not carrying a product), triggers a neighbor check to improve parking space usage.</description></item>
        ///   <item><description><b>Movement halt:</b> Resets acceleration and velocity to stop the mover.</description></item>
        ///   <item><description><b>Motionless detection:</b> If stationary while carrying a product and not yet arrived, increments <c>MotionlessCounter</c>.</description></item>
        /// </list>
        /// </summary>
        private void Arrived()
        {
            if (Center.X >= Destination.X - 0.05 &&
                Center.X <= Destination.X + 0.05 &&
                Center.Y >= Destination.Y - 0.05 &&
                Center.Y <= Destination.Y + 0.05)
            {
                if (Reset)
                    return;

                if (!DestinationUnreachable && ServiceRequester != ActorRefs.Nobody && !Completed)
                {
                    if (UI.Instance.SettingPanel.MQTT.Active)
                        MQTTClient.Instance.PublishComplete(ID);
                    else
                        ServiceRequester.Tell(new TransportCompleted(Transport, TransportDistance));

                    if (UI.Instance.SettingPanel.LogMovers.Active)
                    {
                        Logger!.WriteLine(string.Format(
                         "[{0,-24}] {1,-38} {2,-100}\n",
                         DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                         " - - - - - - - - - - - - ",
                         $"TRANSPORT {Count + 1} COMPLETED IN {Transport} TICK's & TRAVELED {TransportDistance:0.##} mm's FOR [{ServiceRequester.Path.Name}]"
                        ));

                        Logger!.WriteLine(string.Format(
                         "[{0,-24}] {1,-38} {2,-100}",
                         DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                         " - - - - - - - - - - - - ",
                         $"{ServiceRequester.Path.Name} WAITING ON PRODUCTION UNIT COMPLETION"
                        ));
                    }

                    Active = false;

                    var producer = Environment.Instance.Producers.Get(Destination);
                    Completed = producer != null && producer.ProcessingCountdown > 0;
                    Path = [];
                }
                else if (ServiceRequester == ActorRefs.Nobody)
                {
                    if (!Environment.Instance.Parkings.IsParkingSpace(Destination))
                    {
                        Environment.Instance.Parkings.AssignSpace(this);
                        Reset = true;

                        if (UI.Instance.SettingPanel.LogMovers.Active)
                            Logger!.WriteLine(string.Format(
                              "[{0,-24}] {1,-38} {2,-50}",
                              DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                              " - - - - - - - - - - - - ",
                              $"START PARKING ({Destination.X}  {Destination.Y})"
                             ));

                        return;
                    }
                    else if (Environment.Instance.Parkings.CheckNeighbor(this))
                    {
                        Active = true;
                        _parking = _defaultParkCountdown;

                        if (UI.Instance.SettingPanel.LogMovers.Active)
                            Logger!.WriteLine(string.Format(
                                "[{0,-24}] {1,-38} {2,-50}",
                                DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.ffff"),
                                " - - - - - - - - - - - - ",
                                $"CHANGED PARKING ALLOCATION TO  {Destination.X}  {Destination.Y}"
                            ));
                    }
                    else
                        Active = false;
                }
                Acceleration = Vector2.Zero;
                Velocity = Vector2.Zero;
            }
        }

        /// <summary>
        /// Updates the viewport-related rendering parameters for the mover.
        /// <list type="bullet">
        ///   <item><description><b>Scale:</b> Applies the current zoom level to scale the mover’s dimensions.</description></item>
        ///   <item><description><b>Offset:</b> Applies the current screen-space translation for proper positioning.</description></item>
        ///   <item><description><b>Render size:</b> Recalculates the on-screen dimensions of the mover.</description></item>
        ///   <item><description><b>Render radius:</b> Computes a bounding radius used for collision/debug visuals.</description></item>
        ///   <item><description><b>Detection radius:</b> Adjusts the area used for local awareness based on speed and debug settings.</description></item>
        /// </list>
        /// </summary>
        /// <param name="scale">The current rendering scale factor.</param>
        /// <param name="offset">The current screen offset (translation) for rendering.</param>
        internal void UpdateViewport(float scale)
        {
            _renderDimension = Dimension * scale;
            _renderRadius = MathF.Sqrt(_renderDimension.X * _renderDimension.X + _renderDimension.Y * _renderDimension.Y) / 2;
            _renderDetectionRadius = _renderRadius + MaxSpeed * _debugDetection * scale;
        }

        /// <summary>
        /// Applies an absolute debug‐view mask to this mover, clearing any existing
        /// debug draw commands for this ID and then re‐rendering to reflect the new mask.
        /// </summary>
        /// <param name="mask">
        /// A <see cref="DebugFlag"/> bitmask indicating exactly which debug layers
        /// (velocity, acceleration, radius, detection, path) should be active.
        /// </param>
        internal void ChangeDebugView(DebugFlag mask)
        {
            _debug ^= mask;

            Renderer.Instance.RemoveKeyContainedDrawCommand($"6_{ID}");
            Renderer.Instance.RemoveKeyContainedDrawCommand($"7_{ID}");

            Render();
        }

        /// <summary>
        /// Renders the visual representation of the mover, including debug overlays and status indicators.
        /// The following elements may be drawn:
        /// <list type="bullet">
        ///   <item><description><b>Body:</b> Draws the mover as a gray or red-tinged rectangle based on collision state.</description></item>
        ///   <item><description><b>Status light:</b> A circle in the top-right indicating product state: green (active), red (blocked), or yellow (idle).</description></item>
        ///   <item><description><b>ID label:</b> The mover's ID is rendered in white, centered over the rectangle.</description></item>
        ///   <item><description><b>Path:</b> Draws the current movement path as a series of white lines if path debug is enabled.</description></item>
        ///   <item><description><b>Velocity vector:</b> Shows the current direction and magnitude in green if velocity debug is enabled.</description></item>
        ///   <item><description><b>Acceleration vector:</b> Shows acceleration as a cyan line if acceleration debug is enabled.</description></item>
        ///   <item><description><b>Collision radius:</b> Draws a red circular outline representing the mover’s collision bounds if enabled.</description></item>
        ///   <item><description><b>Detection radius:</b> Displays an orange circular area used for local awareness if enabled.</description></item>
        /// </list>
        /// </summary>
        internal void Render()
        {
            var renderPosition = Renderer.Instance.WorldToScreen(Position);
            var renderCenter = renderPosition + _renderDimension / 2;

            DrawMover(renderPosition);
            DrawOLED(renderCenter);
            DrawID(renderCenter);

            if (_debug == DebugFlag.None)
                return;

            if ((_debug & DebugFlag.Velocity) != 0)
                DrawVelocity(renderCenter);

            if ((_debug & DebugFlag.Acceleration) != 0)
                DrawAcceleration(renderCenter);

            if ((_debug & DebugFlag.Radius) != 0)
                DrawRadius(renderCenter);

            if ((_debug & DebugFlag.Detection) != 0)
                DrawDetection(renderCenter);

            if ((_debug & DebugFlag.Path) != 0)
                DrawPathLines(renderCenter);
        }

        /// <summary>
        /// Draws the base rectangle for this mover, colored red if in collision or
        /// disabled, or gray otherwise.
        /// </summary>
        /// <param name="position">Top‐left screen coordinate of the mover’s bounding box.</param>
        private void DrawMover(Vector2 position)
        {
            Renderer.Instance.DrawRectangle(
                id: $"8_{ID}",
                position,
                _renderDimension,
                color: Collided > 0 || Disabled ? Color.Red : Color.GrayLight,
                filled: true,
                RotationAngle
                );
        }

        /// <summary>
        /// Draws the status LED in the top‐right corner of the mover’s rectangle.
        /// Green means active service, red means blocked, yellow means idle.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawOLED(Vector2 center)
        {
            var topRightCornerLED = center;
            topRightCornerLED.X += _renderDimension.X * 0.425f;
            topRightCornerLED.Y -= _renderDimension.Y * 0.425f;

            Renderer.Instance.DrawCircle(
                $"9_{ID}_0",
                topRightCornerLED,
                Math.Min(_renderDimension.X, _renderDimension.Y) * 0.05f,
                ServiceRequester != ActorRefs.Nobody ? Color.Green : State == State.Blocked ? Color.Red : Color.Yellow50
                );
        }

        /// <summary>
        /// Renders the mover’s ID label, centered inside its rectangle.
        /// </summary>
        /// <param name="position">Center point of the mover’s bounding box.</param>
        private void DrawID(Vector2 position)
        {
            Renderer.Instance.DrawText(
                id: $"9_{ID}_1",
                text: ID,
                position: position,
                padding: new(0, 0),
                style: TextStyles.Small,
                color: Color.White,
                center: true,
                UI: false
            );
        }

        /// <summary>
        /// Draws the velocity vector as a line emanating from the mover’s center, if enabled.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawVelocity(Vector2 center)
        {
            var halfDimension = Math.Min(_renderDimension.X, _renderDimension.Y) / 2;
            var renderVectorStart = center + Vector2.Normalize(Velocity) * halfDimension;
            var renderVectorEnd = renderVectorStart + Vector2.Normalize(Velocity) * halfDimension;

            Renderer.Instance.DrawLine(
                $"7_{ID}_2",
                renderVectorStart,
                renderVectorEnd,
                Color.Green,
                _renderDimension.X * 0.1f
                );
        }

        /// <summary>
        /// Draws the acceleration vector as a line from the mover’s center, if enabled.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawAcceleration(Vector2 center)
        {
            var halfDimension =
                Math.Min(_renderDimension.X, _renderDimension.Y) / 2;

            var renderStart =
                center + Vector2.Normalize(Acceleration) * halfDimension;

            var renderEnd =
                renderStart + Vector2.Normalize(Acceleration) * halfDimension;

            Renderer.Instance.DrawLine(
                $"7_{ID}_3",
                renderStart,
                renderEnd,
                Color.Cyan,
                _renderDimension.X * 0.1f
                );
        }

        /// <summary>
        /// Draws a circle around the mover to indicate its collision radius, if enabled.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawRadius(Vector2 center)
        {
            Renderer.Instance.DrawCircle(
                $"7_{ID}_4",
                center,
                _renderRadius,
                Color.Red50,
                false,
                _renderDimension.X * 0.05f
                );
        }

        /// <summary>
        /// Draws a circle around the mover to indicate its detection radius, if enabled.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawDetection(Vector2 center)
        {
            Renderer.Instance.DrawCircle(
                $"7_{ID}_5",
                center,
                _renderDetectionRadius,
                Color.OrangeDark45,
                false,
                _renderDimension.X * 0.05f
                );
        }

        /// <summary>
        /// Draws the planned path as a series of line segments from the mover’s center
        /// through the next <see cref="MovableBody.Path"/> waypoints, if enabled.
        /// </summary>
        /// <param name="center">Center point of the mover’s bounding box.</param>
        private void DrawPathLines(Vector2 center)
        {
            var path = Path.ToArray().Take((int)_pathLookAHead).ToArray();

            if (path.Length > 0)
            {
                var renderStart = center;
                var renderEnd = Renderer.Instance.WorldToScreen(path[0]);

                Renderer.Instance.DrawLine(
                    $"6_{ID}_0",
                    renderStart,
                    renderEnd,
                    Color.White45,
                    10.0f * Renderer.Instance.Scale
                    );

                for (var i = 0; i < path.Length - 1;)
                {
                    renderStart = Renderer.Instance.WorldToScreen(path[i]);
                    renderEnd = Renderer.Instance.WorldToScreen(path[++i]);

                    Renderer.Instance.DrawLine(
                        $"6_{ID}_{i}",
                        renderStart,
                        renderEnd,
                        Color.White45,
                        10.0f * Renderer.Instance.Scale
                        );
                }
                for (var delete = path.Length; delete < _pathLookAHead; delete++)
                    Renderer.Instance.RemoveDrawCommand($"6_{ID}_{delete}");
            }
            else
                Renderer.Instance.RemoveKeyContainedDrawCommand($"6_{ID}");
        }

        /// <summary>
        /// Determines whether a given screen-space point lies within this unit's rendered rectangular bounds.
        /// <list type="bullet">
        ///   <item><description>Transforms the unit’s logical position into screen space using the current scale and offset.</description></item>
        ///   <item><description>Checks if the point is inside the unit’s rendered rectangle using its screen-space position and dimensions.</description></item>
        /// </list>
        /// </summary>
        /// <param name="X">Mouse X coordinate in screen space.</param>
        /// <param name="Y">Mouse Y coordinate in screen space.</param>
        /// <returns><c>true</c> if the point lies within the rendered bounds; otherwise, <c>false</c>.</returns>
        internal bool ScreenPointInsideUnitWorldSpace(Vector2 position)
        {
            if (IsPointInsideRect(position, Renderer.Instance.WorldToScreen(Position), _renderDimension))
                return true;

            return false;
        }

        internal void ResetStats()
        {
            Active = false;
            Disabled = false;
            Completed = true;
            Count = 0;
            Idle = 0;
            Distance = 0;
            FractionalDistance = 0.0f;
            ActiveStationary = 0;
            IdleStationary = 0;
            Transport = 0;
            TransportDistance = 0;
            Acceleration = Vector2.Zero;
            Velocity = Vector2.Zero;
            Destination = Vector2.Zero;
            SwapDestination = Vector2.Zero;
            DestinationUnreachable = false;
            Reset = false;
            Path = [];
            Collided = 0;
        }
    }
}
