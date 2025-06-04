using Akka.Actor;
using Akka.Event;
using Interaction = Simulation.Unit.Interaction;
using Mover = Simulation.UnitTransport.Mover;
using Producer = Simulation.UnitProduction.Producer;
using ProductCompleted = Simulation.Product.ProductCompleted;
using ProductInProgress = Simulation.Product.ProductInProgress;
using ProductRecipe = Simulation.Product.ProductRecipe;
using RemoveWriter = Simulation.Util.RemoveWriter;
using State = Simulation.Unit.State;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Dummy
{
    /// <summary>
    /// Represents a product that undergoes a sequence of interactions (a "recipe") in the simulation.
    /// This actor coordinates its own production lifecycle by managing producer bidding, transport allocation, 
    /// and execution of steps in sequence. It interacts with <see cref="Producer"/> and <see cref="Mover"/> 
    /// actors to simulate production and logistics.
    /// </summary>
    internal class ProductActor : ReceiveActor, IWithTimers
    {
        /// <summary>
        /// Index of the current step in the recipe being processed.
        /// Incremented after each completed step until the recipe is finished.
        /// </summary>
        private int _index = 0;

        /// <summary>
        /// The list of <see cref="Interaction"/>s that define the production steps (the "recipe") this product must follow.
        /// </summary>
        private readonly List<Interaction> _recipe;

        /// <summary>
        /// The currently assigned <see cref="Producer"/> for the active recipe step, if any.
        /// </summary>
        private Producer? _producer;

        /// <summary>
        /// The currently assigned <see cref="Mover"/> responsible for transporting the product to the producer, if any.
        /// </summary>
        private Mover? _mover;

        /// <summary>
        /// Timer support for delayed retries or timed actions.
        /// </summary>
        public ITimerScheduler Timers { get; set; } = null!;

        /// <summary>
        /// Akka.NET logging adapter for writing log messages from within this actor's context.
        /// </summary>
        private readonly ILoggingAdapter _log = Context.GetLogger();

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Reference to the actor responsible for managing transport operations and logistics.</description></item>
        /// </list>
        /// </summary>
        private readonly IActorRef _transportManager;

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Reference to the actor responsible for managing production operations and logistics.</description></item>
        /// </list>
        /// </summary>
        private readonly IActorRef _productionManager;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_transportTicks</b>: Total number of ticks spent transporting the current product.</description></item>
        /// </list>
        /// </summary>
        private ulong _transportTicks = 0;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_transportDistance</b>: Total distance covered while transporting the current product.</description></item>
        /// </list>
        /// </summary>
        private float _transportDistance = 0.0f;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_processingTicks</b>: Total number of ticks spent processing the current product.</description></item>
        /// </list>
        /// </summary>
        private ulong _processingTicks = 0;

        /// <summary>
        /// Initializes a new <see cref="ProductActor"/> with a specific product type.
        /// Loads the product's recipe and enters a waiting state until <see cref="StartProcessing"/> is received.
        /// </summary>
        /// <param name="product">The type of product being produced (e.g., Trimmer, Spinner, Pen).</param>
        public ProductActor(Product.Product product, IActorRef transportManager, IActorRef productionManager)
        {
            _transportManager = transportManager;
            _productionManager = productionManager;

            _recipe = ProductRecipe.Specs[product].Recipe;

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info("INITIALIZED");

            WaitingForStart();
            HandleProductionBailed();
        }

        /// <summary>
        /// Handles special cases like <see cref="KillProduct"/> before delegating to base message handling.
        /// If killed, the actor is reset and its timers are cancelled.
        /// </summary>
        protected override bool AroundReceive(Receive receive, object message)
        {
            if (message is KillProduct)
            {
                if (UI.Instance.SettingPanel.LogProducts.Active)
                    _log.Info("KILLED ON DEMAND");

                Timers.CancelAll();
                Reset();
                return true;
            }
            else
            {
                base.AroundReceive(receive, message);
                return false;
            }
        }

        private void HandleProductionBailed()
        {
            Receive<ProductionBailed>(_ =>
            {
                var txt = $"{_producer!.ID} became blocked";
                _producer?.Queue.Remove(Self.Path.Name);
                _producer = null;

                Retry(txt);
            });
        }

        /// <summary>
        /// Sets up the actor to wait for a <see cref="StartProcessing"/> message, which begins the production process.
        /// </summary>
        private void WaitingForStart()
        {
            Receive<StartProcessing>(_ =>
            {
                ProcessNextStep();
            });
        }

        /// <summary>
        /// Attempts to move to the next step in the recipe. If a producer and transporter are found,
        /// executes the transport to the chosen producer. If no valid candidates are found, retries after delay.
        /// </summary>
        private void ProcessNextStep()
        {
            if (!Cycle.IsRunning || Cycle.IsPaused)
            {
                Retry();
                return;
            }

            if (_index >= _recipe.Count)
            {
                ProductionFinished();
                return;
            }

            if (_producer == null)
            {
                if (!CallForProductionProposal())
                    return;
            }

            if (_mover == null)
            {
                if (!CallForTransportProposal())
                    return;
            }
            ExecuteTransport();
        }

        /// <summary>
        /// Searches for available producers that can handle the current recipe step.
        /// Evaluates proposals based on calculated cost, factoring in transport distance if a mover is already assigned.
        /// Returns the best proposal or retries if none are found.
        /// </summary>
        /// <returns><c>true</c> if a producer was assigned; otherwise, <c>false</c>.</returns>
        private bool CallForProductionProposal()
        {
            var producers = Environment.Instance.Producers.Get(_recipe[_index]);
            List<Producer> proposals = [];
            var min = ulong.MaxValue;

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info($"CALL FOR PROPOSAL {_recipe[_index]}");

            var log = "";

            foreach (var producer in producers)
            {
                if (producer.State != State.Alive || _mover != null && _mover.State != State.Alive)
                    continue;

                ulong cost = 1;
                var producerLog = $"\n[{producer.ID}] ";

                if (_mover != null)
                {
                    var moverCost = _mover.GetCost(producer.Processer.Center, true);

                    if (moverCost != ulong.MaxValue)
                    {
                        producerLog += $"+ [{_mover.ID}]{{ Cost == {moverCost} }} ";
                        cost += moverCost;
                    }
                    else if (moverCost <= 10)
                    {
                        _producer = producer;
                        _producer.AddQueue(Self.Path.Name);

                        if (UI.Instance.SettingPanel.LogProducts.Active)
                            _log.Info($"Already at [{_producer.ID}]");

                        return true;
                    }
                }
                cost = producer.GetDummyCost(_recipe[_index], cost);
                log += $"{producerLog}Cost == {cost}";

                if (cost == ulong.MaxValue)
                    continue;
                else if (cost < min)
                {
                    min = cost;
                    proposals = [];
                    proposals.Add(producer);
                }
                else if (cost == min)
                    proposals.Add(producer);
            }

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info(log);

            if (proposals.Count <= 0)
            {
                Retry($"No available producer's for {_recipe[_index]}");
                return false;
            }

            _producer = proposals[0];
            _productionManager.Tell(new RequestQueueProduction(Self.Path.Name, _producer));
            Become(WaitingForQueueProduction);

            return false;
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Registers a handler for <see cref="ProductionQueued"/> while awaiting production queuing.</description></item>
        /// <item><description>Removes the temporary stacked behavior with <c>Context.UnbecomeStacked()</c> upon receiving the message.</description></item>
        /// <item><description>If <c>msg.Queued</c> is true, logs proposal acceptance (if enabled) and calls <c>ProcessNextStep()</c>.</description></item>
        /// <item><description>If queuing fails, clears the current producer reference and retries with an error message.</description></item>
        /// </list>
        /// </summary>
        private void WaitingForQueueProduction()
        {
            Receive<ProductionQueued>(msg =>
            {
                Context.UnbecomeStacked();

                if (msg.Queued)
                {
                    if (UI.Instance.SettingPanel.LogProducts.Active)
                        _log.Info($"ACCEPTED {_producer!.ID}'s proposal");

                    ProcessNextStep();
                }
                else
                {
                    var txt = _producer!.ID;
                    _producer = null;
                    Retry($"{txt} couldn't be queued");
                }
            });
        }

        /// <summary>
        /// Searches for available <see cref="Mover"/>'s to move the product to the targeted producer.
        /// Assigns the most cost-effective available mover, or retries if none are available.
        /// </summary>
        /// <returns><c>true</c> if a mover was assigned; otherwise, <c>false</c>.</returns>
        private bool CallForTransportProposal()
        {
            var movers = Environment.Instance.Movers.Get();
            List<Mover> proposals = [];
            var min = ulong.MaxValue;

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info("CALL FOR TRANSPORT PROPOSAL");

            var log = "";

            foreach (var mover in movers)
            {
                if (mover.State != State.Alive || mover.ServiceRequester != ActorRefs.Nobody)
                    continue;

                var cost = mover.GetCost(_producer!.Processer.Center);

                if (cost != ulong.MaxValue)
                    log += $"\n[{mover.ID}] Cost == {cost}";

                if (cost == ulong.MaxValue)
                    continue;
                else if (cost < min)
                {
                    min = cost;
                    proposals = [];
                    proposals.Add(mover);
                }
                else if (cost == min)
                    proposals.Add(mover);
            }

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info(log);

            if (proposals.Count <= 0)
            {
                Retry("No avaiblable mover's");
                return false;
            }

            _mover = proposals[0];
            _transportManager.Tell(new RequestTransportAllocation(Self, _mover));
            Become(WaitingForTransportAllocation);

            return false;
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Registers a handler for <see cref="TransportAllocated"/> messages while awaiting transport allocation.</description></item>
        /// <item><description>Removes the temporary behavior from the stack with <c>Context.UnbecomeStacked()</c>.</description></item>
        /// <item><description>If <c>msg.Allocated</c> is true, logs the allocation (if enabled) and invokes <c>ExecuteTransport()</c>.</description></item>
        /// <item><description>If allocation fails, clears the current mover reference and calls <c>Retry()</c> with an error message.</description></item>
        /// </list>
        /// </summary>
        private void WaitingForTransportAllocation()
        {
            Receive<TransportAllocated>(msg =>
            {
                Context.UnbecomeStacked();

                if (msg.Allocated)
                {
                    if (UI.Instance.SettingPanel.LogProducts.Active)
                        _log.Info($"ALLOCATED {_mover!.ID}");

                    ExecuteTransport();
                }
                else
                {
                    var txt = _mover!.ID;
                    _mover = null;
                    Retry($"{txt} couldn't be allocated");
                }
            });
        }

        /// <summary>
        /// Instructs the assigned mover to begin transport to the selected producer.
        /// Transitions the actor to a waiting state for transport completion.
        /// </summary>
        private void ExecuteTransport()
        {
            if (_mover == null ||
                _mover.State != State.Alive ||
                _mover.ServiceRequester == ActorRefs.Nobody ||
                _mover.ServiceRequester.Path != Self.Path)
            {
                var text = "";

                if (_mover == null)
                    text += "Mover ";
                else
                {
                    text += $"[{_mover.ID}] ";

                    if (_mover!.State != State.Alive)
                    {
                        text += "is blocked";
                        _mover.InteractionBailed();
                        _mover.Deallocate();
                    }
                    else if (_mover.ServiceRequester == ActorRefs.Nobody)
                    {
                        text += "is not allocated to a product";
                    }
                    else if (_mover.ServiceRequester.Path != Self.Path)
                        text += $"is not allocated to this product [{Self.Path.Name}]";

                    _mover = null;
                }
                Retry(text);
                return;
            }
            _mover!.StartTransport(_producer!);

            if (UI.Instance.SettingPanel.LogProducts.Active)
                _log.Info("EXECUTE TRANSPORT");

            Become(WaitingForTransportExecution);
        }

        /// <summary>
        /// Waits for <see cref="TransportCompleted"/> before proceeding to the processing step.
        /// Validates producer state before continuing. If the producer died, resets transport and retries.
        /// </summary>
        private void WaitingForTransportExecution()
        {
            Receive<TransportCompleted>(msg =>
            {
                if (_mover != null && _producer != null)
                {
                    _transportTicks += msg.Ticks;
                    _transportDistance += msg.Distance;

                    if (UI.Instance.SettingPanel.LogProducts.Active)
                        _log.Info($"[{_mover!.ID}] INFORM CONFIRMATION after {msg.Ticks} tick's while traveling {msg.Distance:0.##} mm's");

                    Context.UnbecomeStacked();

                    if (_producer.State == State.Alive)
                        ExecuteProcessing();
                    else
                    {
                        var text = $"{_producer.ID} is blocked";
                        _producer.RemoveQueue(Self.Path.Name);
                        _producer = null;
                        _mover.InteractionBailed();
                        Retry(text);
                    }
                }
            });
            HandleProductionBailed();
        }

        /// <summary>
        /// Instructs the producer to begin processing the current recipe step for this product.
        /// Transitions to a waiting state for processing to complete.
        /// </summary>
        private void ExecuteProcessing()
        {
            if (_producer!.ServiceRequester == ActorRefs.Nobody && _producer.State == State.Alive)
            {
                _producer!.StartProcessing(_recipe[_index], Self);

                if (UI.Instance.SettingPanel.LogProducts.Active)
                    _log.Info($"EXECUTE {_recipe[_index]} at {_producer.ID}");

                Become(WaitingForProcessingExecution);
            }
            else if (_producer.ServiceRequester != ActorRefs.Nobody)
                Retry($"{_producer.ID} is busy");
            else
            {
                var text = $"{_producer.ID} is blocked";
                _producer.RemoveQueue(Self.Path.Name);
                _producer = null;
                _mover!.InteractionBailed();
                Retry(text);
            }
        }

        /// <summary>
        /// Waits for <see cref="ProcessingCompleted"/> from the producer. Once received, proceeds to the next recipe step.
        /// Cleans up references and transitions back to <see cref="ProcessNextStep"/>.
        /// </summary>
        private void WaitingForProcessingExecution()
        {
            Receive<ProcessingCompleted>(msg =>
            {
                if (_mover != null && _producer != null)
                {
                    _processingTicks += msg.Ticks;

                    if (UI.Instance.SettingPanel.LogProducts.Active)
                        _log.Info($"[{_producer!.ID}] INFORM CONFIRMATION after {msg.Ticks} tick's");

                    _producer = null;
                    _index++;

                    UnbecomeStacked();
                    ProcessNextStep();
                }
            });
            Receive<TransportCompleted>(msg => { });
            HandleProductionBailed();
        }

        /// <summary>
        /// Finalizes the product's lifecycle after completing all production steps.
        /// Logs completion if product logging is enabled, publishes a <see cref="RemoveWriter"/> message
        /// to dispose of the associated log file, and resets the product actor state.
        /// </summary>
        private void ProductionFinished()
        {
            if (UI.Instance.SettingPanel.LogProducts.Active)
            {
                _mover!.Deallocate();
                _log.Info($"DEALLOCATED {_mover.ID}");
                _log.Info($"PRODUCT COMPLETED  Tick's {{Transport: {_transportTicks}  Interactions: {_processingTicks}}}  Distance: {_transportDistance} mm");
            }

            var actorName = Self.Path.Name;
            var folder = "Products";
            var key = $"{folder}|{actorName}";

            Context.System.EventStream.Publish(new RemoveWriter(key));

            Reset();
        }

        /// <summary>
        /// Cleans up state by deallocating the mover and producer, removing the product from the procedure,
        /// and stopping the actor.
        /// </summary>
        private void Reset()
        {
            if (_mover != null && _mover.ServiceRequester != ActorRefs.Nobody)
            {
                _mover.ServiceRequester = ActorRefs.Nobody;

                if (!Environment.Instance.Parkings.IsParkingSpace(_mover.Destination))
                    _mover.Destination = Vector2.Zero;

                _mover = null;
            }

            if (_producer != null)
            {
                if (_producer.ServiceRequester != ActorRefs.Nobody &&
                    _producer.ServiceRequester.Path.Name == Self.Path.Name)
                {
                    _producer.ProcessingCountdown = 0;
                    _producer.RemoveProcessing();
                }
                _producer.ServiceRequester = ActorRefs.Nobody;
                _producer = null;
            }

            var id = Self.Path.Name;

            if (_index >= _recipe.Count)
                App.ProductSupervisor.Tell(new ProductCompleted(id, _transportTicks, _transportDistance, _processingTicks, $"{_index} / {_index}"));
            else
                App.ProductSupervisor.Tell(new ProductInProgress(id, _transportTicks, _transportDistance, _processingTicks, $"{_index} / {_recipe.Count}"));

            Context.Stop(Self);
        }

        /// <summary>
        /// Logs the specified reason for retry and schedules a retry of the current step after a short delay.
        /// Uses an Akka.NET timer to avoid busy waiting.
        /// </summary>
        /// <param name="log">Message to log before retrying.</param>
        private void Retry(string log = "")
        {
            if (UI.Instance.SettingPanel.LogProducts.Active && log != "")
                _log.Warning(log);

            Timers.StartSingleTimer(
                "RetryTimer",
                new StartProcessing(),
                TimeSpan.FromMilliseconds(Procedure.Instance.ProduceCycle)
                );
        }
    }
}
