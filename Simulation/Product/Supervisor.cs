using Akka.Actor;
using Akka.Event;
using Procedure = Simulation.Dummy.Procedure;
using ProductActor = Simulation.Dummy.ProductActor;
using ProductionManager = Simulation.Dummy.ProductionManager;
using StartProcessing = Simulation.Dummy.StartProcessing;
using TransportManager = Simulation.Dummy.TransportManager;

namespace Simulation.Product
{
    /// <summary>
    /// <list type="bullet">
    /// <item><description>Supervises product actors by spawning a new <b>ProductActor</b> for each <b>AddProduct</b> message.</description></item>
    /// <item><description>Creates a uniquely named child actor based on the product type and ID.</description></item>
    /// <item><description>Immediately sends a <b>StartProcessing</b> message to the newly created product actor.</description></item>
    /// </list>
    /// </summary>
    internal class Supervisor : ReceiveActor
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description>Tracks how many times each <b>Product</b> type has been processed or created.</description></item>
        /// <item><description>Initialized with all <b>Product</b> enum values set to 0.</description></item>
        /// </list>
        /// </summary>
        private readonly Dictionary<Product, ulong> _productCounter =
            Enum.GetValues(typeof(Product))
                      .Cast<Product>()
                      .ToDictionary(
                          prod => prod,
                          count => 0UL
                      );

        /// <summary>
        /// Maps completed product identifiers to their associated tick and distance data.
        /// </summary>
        private readonly Dictionary<string, (ulong Ticks, float Distance, string Interactions)> _assembledProductTracker = [];

        /// <summary>
        /// Maps uncompleted product identifiers to their associated tick and distance data.
        /// </summary>
        private readonly Dictionary<string, (ulong Ticks, float Distance, string Interactions)> _inProgressProductTracker = [];

        /// <summary>
        /// Akka.NET logging adapter for writing log messages from within this actor's context.
        /// </summary>
        private readonly ILoggingAdapter _log = Context.GetLogger();

        /// <summary>
        /// Reference to the actor responsible for managing transport operations and logistics.
        /// </summary>
        private readonly IActorRef _transportManager;

        /// <summary>
        /// Reference to the actor responsible for managing production operations and logistics.
        /// </summary>
        private readonly IActorRef _productionManager;

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Initializes the actor to handle <b>AddProduct</b> messages.</description></item>
        /// <item><description>Creates a <b>ProductActor</b> with a unique name based on product type and ID.</description></item>
        /// <item><description>Sends a <b>StartProcessing</b> message to the new child actor immediately after creation.</description></item>
        /// </list>
        /// </summary>
        public Supervisor()
        {
            _transportManager = Context.ActorOf(
                Props.Create(() => new TransportManager()),
                "TransportBroker"
            );

            _productionManager = Context.ActorOf(
                Props.Create(() => new ProductionManager()),
                "ProductionBroker"
            );

            Receive<CreateProduct>(msg =>
            {
                var count = _productCounter[msg.ProductType];
                var newProductID = count + 1UL;
                _productCounter[msg.ProductType] = newProductID;

                var childName = $"{msg.ProductType}_{newProductID}";

                Context.ActorOf(
                    Props.Create(() => new ProductActor(msg.ProductType, _transportManager, _productionManager)),
                    childName
                )
                .Tell(new StartProcessing());

                if (UI.Instance.SettingPanel.LogProducts.Active)
                    _log.Info("[Supervisor] Received AddProduct(product:{0}), spawned product {1}", msg.ProductType, childName);
            });

            Receive<ProductCompleted>(msg =>
            {
                _assembledProductTracker[msg.Id] = (msg.Ticks + msg.ProcessingTicks, msg.Distance, msg.Interactions);
                Procedure.Instance.Remove();
            });

            Receive<GetCompletedProductTracker>(_ =>
            {
                var snapshot = new Dictionary<string, (ulong, float, string)>(_assembledProductTracker);
                Sender.Tell(new CompletedProductTrackerSnapshot(snapshot));
            });

            Receive<ProductInProgress>(msg =>
            {
                _inProgressProductTracker[msg.Id] = (msg.Ticks + msg.ProcessingTicks, msg.Distance, msg.Interactions);
                Procedure.Instance.Remove();
            });

            Receive<GetInProgressProductTracker>(_ =>
            {
                var snapshot = new Dictionary<string, (ulong, float, string)>(_inProgressProductTracker);
                Sender.Tell(new InProgressProductTrackerSnapshot(snapshot));
            });

            Receive<ResetChildren>(_ =>
            {
                var keep = new HashSet<IActorRef> { _transportManager, _productionManager };

                foreach (var child in Context.GetChildren())
                    if (!keep.Contains(child))
                        child.Tell(PoisonPill.Instance);

                Procedure.Instance.Count = 0;
            });

            Receive<ResetSupervisorState>(_ =>
            {
                var keep = new HashSet<IActorRef> { _transportManager, _productionManager };

                foreach (var child in Context.GetChildren())
                    if (!keep.Contains(child))
                        child.Tell(PoisonPill.Instance);

                _assembledProductTracker.Clear();
                _inProgressProductTracker.Clear();

                foreach (var key in _productCounter.Keys.ToList())
                    _productCounter[key] = 0UL;

                Procedure.Instance.Count = 0;
            });
        }
    }
}
