using Akka.Actor;
using Akka.Event;
using I4Sim = Simulation.MQTT.Messages.I4Sim;
using JsonSerializer = System.Text.Json.JsonSerializer;
using Procedure = Simulation.Dummy.Procedure;
using ResetSupervisorState = Simulation.Product.ResetSupervisorState;
using State = Simulation.Unit.State;
using StateChange = Simulation.MQTT.Messages.StateChange;

namespace Simulation.MQTT
{
    /// <summary>
    /// Facade for interacting with the MQTT client actor system.
    /// Provides a thread-safe singleton instance for sending Start and Stop commands
    /// to the underlying <see cref="ClientActor"/>, which handles MQTT communication.
    /// </summary>
    /// <param name="actor">
    /// <list type="bullet">
    /// <item><description>Reference to the <b>ClientActor</b> responsible for handling MQTT operations and message flow.</description></item>
    /// </list>
    /// </param>
    internal class Client(IActorRef actor)
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_instance</b>: Singleton instance of the <b>Client</b> used to ensure only one instance exists.</description></item>
        /// </list>
        /// </summary>
        private static Client? _instance;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_lock</b>: Synchronization object used to make singleton access thread-safe.</description></item>
        /// </list>
        /// </summary>
        private static readonly object _lock = new();

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_actor</b>: Reference to the actor that handles communication or control logic for the client.</description></item>
        /// </list>
        /// </summary>
        private readonly IActorRef _actor = actor;

        /// <summary>
        /// Gets the singleton instance of the Client.
        /// Throws an exception if the client has not yet been initialized.
        /// </summary>
        internal static Client Instance
        {
            get => _instance
                   ?? throw new InvalidOperationException(
                         "MQTTClient not initialized; call Initialize(...) first");
        }

        /// <summary>
        /// Initializes the MQTT client by creating the underlying ClientActor
        /// and storing the instance. Thread-safe and idempotent.
        /// </summary>
        /// <param name="environment">The environment identifier, used as the MQTT ClientId.</param>
        /// <param name="brokerIP">The MQTT broker IP address. Defaults to WSL IP.</param>
        internal static void Initialize(string environment = "i4sim",
                                        string brokerIP = "172.26.47.34")
        {
            lock (_lock)
            {
                if (_instance != null) return;

                App.Log.Info("[Client] Initialized MQTT ClientActor(environment: {0}, ip: {1})", environment, brokerIP);
                var mqttActor = App.System.ActorOf(
                    Props.Create(() => new ClientActor(environment, brokerIP)),
                    "MQTT");

                _instance = new Client(mqttActor);
                App.Log.Info("[Client] Initialized itself with MQTT ClientActor: {0}", mqttActor.Path);
            }
        }

        /// <summary>
        /// Sends a StartClient message to the MQTT actor, which will initialize
        /// the MQTT connection and begin creating simulation units.
        /// </summary>
        internal Task Start()
        {
            Cycle.Halt();

            while (Procedure.Instance.Count > 0)
            {
                App.ProductSupervisor.Tell(new ResetSupervisorState());
                Environment.Instance.Movers.FullReset(true);
                Environment.Instance.Producers.FullReset(true);
            }

            Environment.Instance.Collisions = 0;

            Cycle.StartedAt = DateTime.Now;
            Cycle.ResetTickFetch();
            Cycle.StartRenderer();
            Cycle.Start();

            App.Log.Info("[Client] sent StartClient message to MQTT ClientActor");
            _actor.Tell(new StartClient());
            return Task.CompletedTask;
        }

        /// <summary>
        /// Sends a StopClient message to the MQTT actor, which will shut down the
        /// MQTT connection and stop message processing.
        /// </summary>
        internal Task Stop()
        {
            App.Log.Info("[Client] Sent StopClient message to MQTT ClientActor");
            _actor.Tell(new StopClient());

            Environment.Instance.Movers.Reset();
            Environment.Instance.Producers.Reset();

            Environment.Instance.Collisions = 0;

            return Task.CompletedTask;
        }

        /// <summary>
        /// Publishes a state change message for a given entity name and state to the actor system.
        /// </summary>
        /// <param name="name">The name of the entity whose state has changed.</param>
        /// <param name="state">The new state to be published.</param>
        /// <returns>A completed task representing the publish operation.</returns>
        internal Task PublishStateChange(string name, State state)
        {
            var msg = new StateChange(name, state);
            var payload = JsonSerializer.Serialize(msg);

            App.Log.Info("[Client] Sent StateChange(name: {0}, state: {1}) publish message to MQTT actor on payload {2}", name, state, payload);
            _actor.Tell(new Publish(I4Sim.StateChange(name), payload));
            return Task.CompletedTask;
        }

        /// <summary>
        /// Publishes a Complete message for the given unit.
        /// </summary>
        internal Task PublishComplete(string name)
        {
            App.Log.Info("[Client] Sent StartComplete(name: {0}) publish message to MQTT actor", name);
            _actor.Tell(new StartComplete(name));
            return Task.CompletedTask;
        }
    }
}
