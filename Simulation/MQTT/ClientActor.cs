using Akka.Actor;
using Akka.Event;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Extensions.ManagedClient;
using Acknowledge = Simulation.MQTT.Messages.Acknowledge;
using Create = Simulation.MQTT.Messages.Create;
using Encoding = System.Text.Encoding;
using I4Sim = Simulation.MQTT.Messages.I4Sim;
using Interaction = Simulation.Unit.Interaction;
using InteractionExtensions = Simulation.Unit.InteractionExtensions;
using JsonSerializer = System.Text.Json.JsonSerializer;
using Perform = Simulation.MQTT.Messages.Perform;
using Purge = Simulation.MQTT.Messages.Purge;
using RequestCost = Simulation.MQTT.Messages.RequestCost;
using ResponseCost = Simulation.MQTT.Messages.ResponseCost;
using State = Simulation.Unit.State;

namespace Simulation.MQTT
{
    /// <summary>
    /// Actor responsible for managing the MQTT connection lifecycle, subscriptions, 
    /// and ordered creation of simulation units with acknowledgement and retry logic.
    /// </summary>
    internal class ClientActor : ReceiveActor
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_actor</b>: Reference to the active unit creation actor handling current simulation logic.</description></item>
        /// </list>
        /// </summary>
        private IActorRef? _actor;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_client</b>: Managed MQTT client responsible for connecting, subscribing, and publishing messages.</description></item>
        /// </list>
        /// </summary>
        private readonly IManagedMqttClient _client;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_options</b>: Configuration options for initializing and managing the MQTT client.</description></item>
        /// </list>
        /// </summary>
        private readonly ManagedMqttClientOptions _options;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_log</b>: Akka.NET logging adapter for internal diagnostics and debugging output.</description></item>
        /// </list>
        /// </summary>
        private readonly ILoggingAdapter _log = Context.GetLogger();

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_pendingCreates</b>: Queue of unit creation requests waiting to be processed in order.</description></item>
        /// </list>
        /// </summary>
        private Queue<Create>? _pendingCreates;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>ClientActor</b>: Initializes the MQTT client actor with connection options and message handlers.</description></item>
        /// <item><description>Configures MQTT connection using provided <b>clientID</b> and <b>brokerIP</b>.</description></item>
        /// <item><description>Initializes MQTT client with auto-reconnect behavior.</description></item>
        /// <item><description>Logs initialization details for debugging and traceability.</description></item>
        /// <item><description>Registers handlers for incoming MQTT messages, client lifecycle events, and unit creation/control messages.</description></item>
        /// <item><description>Supports creation and completion flows using actor-based sequencing and retry logic.</description></item>
        /// </list>
        /// </summary>
        /// <param name="clientID">Unique MQTT client identifier (ClientId).</param>
        /// <param name="brokerIP">IP address of the MQTT broker.</param>
        public ClientActor(string clientID, string brokerIP)
        {
            _options = new ManagedMqttClientOptionsBuilder()
                .WithAutoReconnectDelay(TimeSpan.FromSeconds(5))
                .WithClientOptions(new MqttClientOptionsBuilder()
                    .WithClientId(clientID)
                    .WithTcpServer(brokerIP)
                    .Build())
                .Build();

            _client = new MqttFactory().CreateManagedMqttClient();

            _log.Info("Initialized with client ID {0} and broker IP {1}", clientID, brokerIP);

            Receive<MqttMessage>(msg =>
            {
                if (UI.Instance.SettingPanel.LogMQTT.Active)
                    _log.Info($"[{msg.Topic}] {msg.Payload}");

                HandleAcknowledgements(msg);
                HandleRequestCost(msg);
                HandlePerform(msg);
            });
            ReceiveAsync<StartClient>(async _ => await DoStart());
            ReceiveAsync<StopClient>(async _ => await DoStop());
            ReceiveAsync<Publish>(async p =>
            {
                if (UI.Instance.SettingPanel.LogMQTT.Active)
                    _log.Info("[{0}] {1}", p.Topic, p.Payload);

                await PublishRaw(p).ConfigureAwait(false);
            });
            ReceiveAsync<Subscribe>(async s => await _client.SubscribeAsync(s.Topic).ConfigureAwait(false));

            Receive<StartSequence>(_ => SpawnNextCreator());
            Receive<CreateSucceeded>(_ => OnChildDone());
            Receive<CreateFailed>(_ => OnChildFailed());

            Receive<StartComplete>(sc =>
                Context.ActorOf(Props.Create(() => new UnitCompleteActor(Self, sc.Name)))
            );
            Receive<CompleteSucceeded>(cs =>
            {
                if (UI.Instance.SettingPanel.LogMQTT.Active)
                    _log.Info("Complete for {0} ACK’d", cs.Name);
            });
            Receive<CompleteFailed>(cf =>
            {
                if (UI.Instance.SettingPanel.LogMQTT.Active)
                    _log.Warning("Complete for {0} failed after retries", cf.Name);
            });
        }

        /// <summary>
        /// Called when the actor is started. Captures the actor reference and
        /// attaches the raw MQTT callback to forward messages into the actor mailbox.
        /// </summary>
        protected override void PreStart()
        {
            base.PreStart();

            _actor = Self;

            _client.ApplicationMessageReceivedAsync += e =>
            {
                var topic = e.ApplicationMessage.Topic;
                var payload = Encoding.UTF8.GetString(e.ApplicationMessage.PayloadSegment);

                _actor.Tell(new MqttMessage(topic, payload));
                return Task.CompletedTask;
            };
        }

        /// <summary>
        /// Starts the MQTT client, subscribes to required topics and enqueues
        /// all pending Create messages into the internal queue before beginning sequence.
        /// </summary>
        private async Task DoStart()
        {
            await _client.StartAsync(_options).ConfigureAwait(false);

            await _client.SubscribeAsync("i4sim/create/ack");
            await _client.SubscribeAsync("i4sim/+/requestCost");
            await _client.SubscribeAsync("i4sim/+/perform");
            await _client.SubscribeAsync("i4sim/+/complete/ack");

            var allCreates = Environment.Instance.Movers.Get()
                .Select(m => new Create(
                    m.ID,
                    m.ID,
                    $"{m.Model}",
                    [InteractionExtensions.ToUrl(Interaction.Transport)],
                    m.State
                ))
                .Concat(Environment.Instance.Producers.Get()
                .Select(p =>
                {
                    var ints = p.InteractionCost.Keys
                                  .Select(InteractionExtensions.ToUrl)
                                  .ToList();
                    return new Create(
                        p.ID,
                        p.ID,
                        $"{p.Model}",
                        ints,
                        p.State
                    );
                }))
                .ToList();

            _pendingCreates = new Queue<Create>(allCreates)!;
            Self.Tell(new StartSequence());
        }

        /// <summary>
        /// Stops the MQTT client by publishing a Purge message, waiting briefly,
        /// then stopping the client and disconnecting.
        /// </summary>
        private async Task DoStop()
        {
            await PublishRaw(new Publish(I4Sim.Purge, JsonSerializer.Serialize(new Purge())));
            await Task.Delay(16);

            if (_client.IsStarted)
                await _client.StopAsync();
        }

        /// <summary>
        /// Handles incoming MQTT acknowledgment messages. If the message topic matches the create acknowledgment topic,
        /// it deserializes the payload and publishes the acknowledgment to the actor system's event stream.
        /// </summary>
        /// <param name="msg">The incoming MQTT message.</param>
        private static void HandleAcknowledgements(MqttMessage msg)
        {
            var parts = msg.Topic.Split('/');

            if (parts.Length == 4
               && parts[0] == "i4sim"
               && parts[2] == "complete"
               && parts[3] == "ack")
            {
                if (!parts[1].Contains("APM"))
                {
                    var producer = Environment.Instance.Producers.Get(parts[1]);
                    producer?.CompleteProcessing();
                }

                var ack = JsonSerializer.Deserialize<Acknowledge>(msg.Payload)
                          ?? new Acknowledge(parts[1]);
                Context.System.EventStream.Publish(ack);
            }
            else if (msg.Topic == I4Sim.CreateAck)
            {
                var ack = JsonSerializer.Deserialize<Acknowledge>(msg.Payload)!;
                Context.System.EventStream.Publish(ack);
            }
        }

        /// <summary>
        /// Handles incoming cost request messages over MQTT for a given unit. Parses the unit ID and requested interaction
        /// from the topic and payload, calculates the cost based on the unit type (producer or mover),
        /// and publishes a corresponding cost response message.
        /// </summary>
        /// <param name="msg">The incoming MQTT message containing the cost request.</param>
        private void HandleRequestCost(MqttMessage msg)
        {
            var parts = msg.Topic.Split('/');

            if (parts.Length == 3 && parts[0] == "i4sim" && parts[2] == "requestCost")
            {
                ulong costValue = 0;
                var unitID = parts[1];
                var dto = JsonSerializer.Deserialize<RequestCost>(msg.Payload)!;
                var interaction = InteractionExtensions.FromUrl(dto.InteractionElement);

                if (interaction == Interaction.Transport && dto.Destination != null)
                {
                    var mover = Environment.Instance.Movers.Get(unitID);
                    var producer = Environment.Instance.Producers.Get(dto.Destination);

                    if (mover != null && producer != null && mover.State == State.Alive)
                        costValue = mover.GetCost(producer.Processer.Position);
                }
                else
                {
                    var producer = Environment.Instance.Producers.Get(unitID);

                    if (producer != null && producer.State == State.Alive)
                        costValue = producer.GetMQTTCost(interaction);
                }

                if (costValue > 0)
                {
                    var resp = new ResponseCost(costValue);
                    var payload = JsonSerializer.Serialize(resp);
                    Self.Tell(new Publish(I4Sim.ResponseCost(unitID), payload));
                }
            }
        }

        /// <summary>
        /// Handles incoming MQTT "perform" messages for a specific unit. Parses the unit ID from the topic,
        /// deserializes the payload into a <c>Perform</c> DTO and triggers the appropriate action on the unit
        /// (e.g., start processing for producers or start transport for movers). 
        /// Sends an acknowledgment message back to the broker after handling.
        /// </summary>
        /// <param name="msg">The incoming MQTT message containing the perform request.</param>
        private void HandlePerform(MqttMessage msg)
        {
            var parts = msg.Topic.Split('/');

            if (parts.Length == 3 && parts[0] == "i4sim" && parts[2] == "perform")
            {
                var unitID = parts[1];
                var dto = JsonSerializer.Deserialize<Perform>(msg.Payload)!;
                var interaction = InteractionExtensions.FromUrl(dto.InteractionElement);

                if (interaction == Interaction.Transport && dto.Destination != null)
                {
                    var mover = Environment.Instance.Movers.Get(unitID);
                    var producer = Environment.Instance.Producers.Get(dto.Destination);

                    if (mover != null && mover.State == State.Alive && producer != null)
                        mover.StartTransport(producer);
                    else
                        return;
                }
                else
                {
                    var producer = Environment.Instance.Producers.Get(unitID);

                    if (producer != null && producer.State == State.Alive)
                        producer?.StartProcessing(interaction, Self);
                    else
                        return;
                }
                var ackPayload = JsonSerializer.Serialize(new Acknowledge(unitID));
                Self.Tell(new Publish(I4Sim.PerformAck(unitID), ackPayload));
            }
        }

        /// <summary>
        /// Helper to enqueue a raw MQTT publish to the managed client.
        /// </summary>
        private Task PublishRaw(Publish p) => _client.EnqueueAsync(
            new MqttApplicationMessageBuilder()
               .WithTopic(p.Topic)
               .WithPayload(p.Payload)
               .Build());

        /// <summary>
        /// Dequeues the next Create message and spawns a <see cref="UnitCreatorActor"/>
        /// child to send and await its acknowledgement.
        /// </summary>
        private void SpawnNextCreator()
        {
            if (_pendingCreates is null || _pendingCreates.Count == 0)
            {
                if (UI.Instance.SettingPanel.LogMQTT.Active)
                    _log.Info("ALL UNITS CREATED!");

                return;
            }

            var next = _pendingCreates.Dequeue();

            if (UI.Instance.SettingPanel.LogMQTT.Active)
                _log.Info($"Spawning creator supervisor for {next.Name}");

            Context.ActorOf(
                Props.Create(() => new UnitCreatorActor(Self, next)),
                $"{next.Name}"
            );
        }

        /// <summary>
        /// Called when a child <see cref="UnitCreatorActor"/> reports success or failure.
        /// Triggers the next sequence iteration.
        /// </summary>
        private void OnChildDone()
        {
            if (UI.Instance.SettingPanel.LogMQTT.Active)
                _log.Info("Unit creation done, proceeding to next");

            Self.Tell(new StartSequence());
        }

        /// <summary>
        /// Handles the failure of a child <see cref="UnitCreatorActor"/> during the creation sequence.
        /// Stops the MQTT client, triggers a purge of the simulation state and deactivates bidding.
        /// </summary>
        private void OnChildFailed()
        {
            if (UI.Instance.SettingPanel.LogMQTT.Active)
                _log.Info("Unit creation failed, PURGING STATE ACTIVATED!");

            Client.Instance.Stop();
        }
    }
}
