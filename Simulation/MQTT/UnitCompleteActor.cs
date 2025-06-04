using Akka.Actor;
using Acknowledge = Simulation.MQTT.Messages.Acknowledge;
using Complete = Simulation.MQTT.Messages.Complete;
using I4Sim = Simulation.MQTT.Messages.I4Sim;
using JsonSerializer = System.Text.Json.JsonSerializer;

namespace Simulation.MQTT
{
    /// <summary>
    /// An Akka.NET actor responsible for completing a unit's task via MQTT messaging.
    /// Sends a completion message and waits for an acknowledgment. If no acknowledgment
    /// is received within a timeout, the message is retried up to 5 times.
    /// </summary>
    internal class UnitCompleteActor : ReceiveActor
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_mqtt</b>: Reference to the MQTT actor for sending completion notifications or updates.</description></item>
        /// </list>
        /// </summary>
        private readonly IActorRef _mqtt;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_name</b>: Identifier for the unit or actor instance being tracked for completion.</description></item>
        /// </list>
        /// </summary>
        private readonly string _name;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_maxAttempts</b>: Maximum number of allowed retry attempts for confirming unit completion.</description></item>
        /// </list>
        /// </summary>
        private readonly int _maxAttempts = 10;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_attempts</b>: Current number of attempts made to confirm or handle unit completion.</description></item>
        /// </list>
        /// </summary>
        private int _attempts;

        /// <summary>
        /// Initializes a new instance of the <see cref="UnitCompleteActor"/> class, subscribing to acknowledgment messages
        /// and setting up message handlers for the completion workflow.
        /// </summary>
        /// <param name="mqtt">The MQTT actor used to publish messages.</param>
        /// <param name="name">The name or ID of the unit for which completion is being tracked.</param>
        public UnitCompleteActor(IActorRef mqtt, string name)
        {
            _mqtt = mqtt;
            _name = name;
            _attempts = 0;

            Context.System.EventStream.Subscribe(Self, typeof(Acknowledge));

            Receive<StartComplete>(_ => SendComplete());
            Receive<Acknowledge>(ack => HandleAck(ack));
            Receive<ReceiveTimeout>(_ => Retry());
        }

        /// <summary>
        /// Sends a completion message via MQTT and sets a receive timeout for retry logic.
        /// </summary>
        private void SendComplete()
        {
            var msg = new Complete();
            var payload = JsonSerializer.Serialize(msg);

            _mqtt.Tell(new Publish(I4Sim.Complete(_name), payload));
            _attempts++;
            Context.SetReceiveTimeout(TimeSpan.FromMilliseconds(500));
        }

        /// <summary>
        /// Handles acknowledgment messages. If the acknowledgment matches this unit's name,
        /// the completion is considered successful and reported to the parent.
        /// </summary>
        /// <param name="ack">The acknowledgment message received from MQTT.</param>
        private void HandleAck(Acknowledge ack)
        {
            if (ack.Name == _name)
            {
                Context.SetReceiveTimeout(null);
                Context.Parent.Tell(new CompleteSucceeded(_name));
                Context.Stop(Self);
            }
        }

        /// <summary>
        /// Retries sending the completion message if no acknowledgment has been received within the timeout period.
        /// After max unsuccessful attempts, the failure is reported to the parent actor.
        /// </summary>
        private void Retry()
        {
            if (_attempts < _maxAttempts)
            {
                SendComplete();
            }
            else
            {
                Context.SetReceiveTimeout(null);
                Context.Parent.Tell(new CompleteFailed(_name));
                Context.Stop(Self);
            }
        }

        /// <summary>
        /// Automatically triggers the completion process when the actor starts.
        /// </summary>
        protected override void PreStart() => Self.Tell(new StartComplete(_name));
    }
}
