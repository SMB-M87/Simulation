using Akka.Actor;
using Acknowledge = Simulation.MQTT.Messages.Acknowledge;
using Create = Simulation.MQTT.Messages.Create;
using I4Sim = Simulation.MQTT.Messages.I4Sim;
using JsonSerializer = System.Text.Json.JsonSerializer;

namespace Simulation.MQTT
{
    /// <summary>
    /// <list type="bullet">
    /// <item><description><b>UnitCreatorActor</b>: Handles unit creation logic with retry support using Akka.NET's actor model.</description></item>
    /// <item><description>Communicates with the MQTT actor to publish unit creation results or status updates.</description></item>
    /// <item><description>Uses a maximum number of attempts to retry creation logic before giving up.</description></item>
    /// </list>
    /// </summary>
    internal class UnitCreatorActor : ReceiveActor
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_mqtt</b>: Reference to the MQTT actor used for publishing messages related to unit creation.</description></item>
        /// </list>
        /// </summary>
        private readonly IActorRef _mqtt;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_create</b>: Contains the unit creation request data and configuration.</description></item>
        /// </list>
        /// </summary>
        private readonly Create _create;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_maxAttempts</b>: Maximum number of retry attempts allowed for unit creation.</description></item>
        /// </list>
        /// </summary>
        private readonly int _maxAttempts = 10;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_attempts</b>: Tracks the current number of unit creation attempts.</description></item>
        /// </list>
        /// </summary>
        private int _attempts;

        /// <summary>
        /// Constructs a UnitCreatorActor responsible for publishing a Create message
        /// and handling its acknowledgement from the MQTT broker.
        /// </summary>
        /// <param name="mqtt">Reference to the MQTT actor that handles publishing.</param>
        /// <param name="create">The Create message to publish.</param>
        public UnitCreatorActor(IActorRef mqtt, Create create)
        {
            _mqtt = mqtt;
            _create = create;
            _attempts = 0;

            Context.System.EventStream.Subscribe(Self, typeof(Acknowledge));

            Receive<StartCreate>(_ => SendCreate());
            Receive<Acknowledge>(ack => HandleAck(ack));
            Receive<ReceiveTimeout>(_ => Retry());
        }

        /// <summary>
        /// Publishes the Create message via the MQTT actor and sets a timeout for retry.
        /// </summary>
        private void SendCreate()
        {
            var payload = JsonSerializer.Serialize(_create);
            _mqtt.Tell(new Publish(I4Sim.Create, payload));
            _attempts++;
            Context.SetReceiveTimeout(TimeSpan.FromMilliseconds(500));
        }

        /// <summary>
        /// Handles an Acknowledge message and stops the actor if it matches the unit name.
        /// </summary>
        private void HandleAck(Acknowledge ack)
        {
            if (ack.Name == _create.Name)
            {
                Context.SetReceiveTimeout(null);
                Context.Parent.Tell(new CreateSucceeded(_create.Name));
                Context.Stop(Self);
            }
        }

        /// <summary>
        /// Retries the Create message if no ACK is received within the timeout.
        /// Stops the actor after max failed attempts.
        /// </summary>
        private void Retry()
        {
            if (_attempts < _maxAttempts)
            {
                SendCreate();
            }
            else
            {
                Context.SetReceiveTimeout(null);
                Context.Parent.Tell(new CreateFailed(_create.Name));
                Context.Stop(Self);
            }
        }

        /// <summary>
        /// Automatically triggers the StartCreate message when the actor starts.
        /// </summary>
        protected override void PreStart() => Self.Tell(new StartCreate(_create));

        /// <summary>
        /// Called when the actor is stopped. Unsubscribes from the event stream to prevent memory leaks or dangling references.
        /// </summary>
        protected override void PostStop() { Context.System.EventStream.Unsubscribe(Self, typeof(Acknowledge)); base.PostStop(); }
    }
}
