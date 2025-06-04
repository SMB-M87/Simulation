using Akka.Actor;
using Akka.Event;
using CreateProduct = Simulation.Product.CreateProduct;
using Stopwatch = System.Diagnostics.Stopwatch;

namespace Simulation.Dummy
{
    /// <summary>
    /// Controls the simulated product generation logic in the simulation.
    /// Periodically spawns new <see cref="ProductActor"/>s according to available capacity and cycle rate.
    /// </summary>
    internal class Procedure
    {
        /// <summary>
        /// Random number generator used for simulating stochastic behavior in product generation.
        /// </summary>
        private readonly Random _random;

        /// <summary>
        /// The maximum number of products allowed at any given time.
        /// Used to limit spawning based on simulation constraints.
        /// </summary>
        internal uint MaxProducts { get; set; }

        /// <summary>
        /// Tracks the total number of active or spawned items in the simulation.
        /// </summary>
        internal uint Count { get; set; }

        /// <summary>
        /// Indicates whether the bidding and generation loop is currently active.
        /// Marked as volatile to allow thread-safe access.
        /// </summary>
        private volatile bool _running;

        /// <summary>
        /// Synchronization lock for managing thread-safe operations inside the bidding loop.
        /// </summary>
        private readonly object _syncLock = new();

        /// <summary>
        /// Background thread used to run the periodic bidding and product spawning logic.
        /// </summary>
        private Thread? _biddingThread;

        /// <summary>
        /// Gets or sets the product generation interval in milliseconds.
        /// </summary>
        internal double ProduceCycle
        {
            get
            {
                return _produceCycle;
            }
            set
            {
                _produceCycle = value * _produceDefault;
            }
        }
        private double _produceCycle;
        private const double _produceDefault = 200;

        /// <summary>
        /// Singleton instance of the <see cref="Procedure"/> class, used to coordinate bidding and product generation globally.
        /// </summary>
        private static Procedure? _instance;

        /// <summary>
        /// Lock object to ensure thread-safe access and initialization of the <see cref="_instance"/>.
        /// </summary>
        private static readonly object _lock = new();

        /// <summary>
        /// Singleton access to the <see cref="Procedure"/> instance.
        /// Must be initialized using <see cref="Initialize"/>.
        /// </summary>
        internal static Procedure Instance
        {
            get
            {
                if (_instance == null)
                    throw new InvalidOperationException("[Procedure] Is not initialized, call the Procedure.Initialize(uint: MaxProducts) function.");

                return _instance;
            }
        }

        /// <summary>
        /// Initializes the singleton <see cref="Procedure"/> instance.
        /// </summary>
        internal static void Initialize()
        {
            lock (_lock)
            {
                _instance ??= new Procedure();
            }
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Procedure"/> class.
        /// Sets up the product generation counters and assigns the actor system.
        /// The maximum allowed products is based on the number of available movers in the environment.
        /// </summary>
        private Procedure()
        {
            Count = 0;
            MaxProducts = 0;

            _random = new();
            _produceCycle = 16.66 * _produceDefault;

            App.Log.Info("[Procedure] Initialization completed with a max product count of {0}", MaxProducts);
        }

        /// <summary>
        /// Configures the procedure’s product generation parameters.
        /// </summary>
        /// <param name="cycle">
        ///   The base generation interval (in milliseconds) before any internal scaling.
        ///   This value will be multiplied by the internal factor to arrive at the actual produce cycle.
        /// </param>
        /// <param name="maxProducts">
        ///   The maximum number of concurrent products that may exist in the simulation.
        ///   Once this limit is reached, no further products will be spawned until existing ones are removed.
        /// </param>
        internal void Setup(double cycle, uint maxProducts)
        {
            ProduceCycle = 1000.0f / cycle;
            MaxProducts = maxProducts;
        }

        /// <summary>
        /// Starts the background bidding thread, allowing product generation to begin.
        /// </summary>
        internal void Start()
        {
            if (_running)
                return;

            App.Log.Info("[Procedure] Started the bidding loop");

            _running = true;
            _biddingThread = new Thread(BiddingLoop) { IsBackground = true };
            _biddingThread.Start();
        }

        /// <summary>
        /// Stops the bidding thread and halts all product generation.
        /// </summary>
        internal void Stop()
        {
            App.Log.Info("[Procedure] Stopped the bidding loop");

            _running = false;
            _biddingThread?.Join();
        }

        /// <summary>
        /// Decreases the internal product counter, allowing another product to be spawned if below max.
        /// </summary>
        internal void Remove()
        {
            if (Count <= 0)
                return;

            var prev = Count;
            Count--;

            if (UI.Instance.SettingPanel.LogProducts.Active)
                App.Log.Warning("[Procedure] Decreased the product count from {0} to {1}", prev, Count);
        }

        /// <summary>
        /// Loop that periodically attempts to spawn new <see cref="ProductActor"/>s at a fixed interval (_produceCycle).
        /// If the simulation is not paused and the current product count is below the allowed maximum, a new product is added.
        /// Runs continuously in a background thread while <c>_running</c> is true.
        /// </summary>
        private void BiddingLoop()
        {
            var stopwatch = Stopwatch.StartNew();
            var lastUpdateTime = stopwatch.Elapsed.TotalMilliseconds;

            while (_running)
            {
                var currentTime = stopwatch.Elapsed.TotalMilliseconds;
                var deltaTime = currentTime - lastUpdateTime;

                if (deltaTime >= _produceCycle)
                {
                    if (!Cycle.IsPaused)
                        lock (_syncLock)
                            if (Count < MaxProducts)
                                AddProduct();

                    lastUpdateTime = currentTime;
                }

                Thread.Yield();
            }
        }

        /// <summary>
        /// Creates a new <see cref="ProductActor"/> with a randomly selected product type (Trimmer, Spinner, or Pen),
        /// assigns it a unique ID based on its type, increments counters accordingly and sends a <see cref="CreateProduct"/> message
        /// to the product supervisor.
        /// </summary>
        private void AddProduct()
        {
            Count++;
            var producers = Enum.GetValues<Product.Product>();
            var product = producers[_random.Next(producers.Length)];

            App.ProductSupervisor.Tell(new CreateProduct(product));
        }
    }
}
