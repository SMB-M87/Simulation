using Create = Simulation.MQTT.Messages.Create;

namespace Simulation.MQTT
{
    /// <summary>
    /// Message to request the MQTT client to start, connect to the broker,
    /// subscribe to topics and begin unit creation.
    /// </summary>
    internal sealed record StartClient();

    /// <summary>
    /// <list type="bullet">
    /// <item><description><b>StartSequence</b>: Message used to trigger the start of the unit creation sequence within the client actor.</description></item>
    /// </list>
    /// </summary>
    internal sealed record StartSequence();

    /// <summary>
    /// Message to request the MQTT client to stop, publish a purge,
    /// and disconnect from the broker.
    /// </summary>
    internal sealed record StopClient();

    /// <summary>
    /// Message to subscribe to an MQTT topic.
    /// </summary>
    /// <param name="Topic">The topic string to subscribe to.</param>
    internal sealed record Subscribe(string Topic);

    /// <summary>
    /// Message to publish a payload to a specific MQTT topic.
    /// </summary>
    /// <param name="Topic">The topic string to publish to.</param>
    /// <param name="Payload">The serialized payload to publish.</param>
    internal sealed record Publish(string Topic, string Payload);

    /// <summary>
    /// Internal message used to forward raw MQTT messages into the actor system.
    /// </summary>
    /// <param name="Topic">The topic on which the message was received.</param>
    /// <param name="Payload">The UTF-8 string representation of the payload.</param>
    internal sealed record MqttMessage(string Topic, string Payload);

    /// <summary>
    /// Message sent to a UnitCreatorActor to start sending a Create message.
    /// </summary>
    /// <param name="Create">The Create message payload to send.</param>
    internal sealed record StartCreate(Create Create);

    /// <summary>
    /// Message indicating that a Create message was acknowledged successfully.
    /// </summary>
    /// <param name="Name">The name of the unit that was acknowledged.</param>
    internal sealed record CreateSucceeded(string Name);

    /// <summary>
    /// Message indicating that a Create message failed to be acknowledged after retries.
    /// </summary>
    /// <param name="Name">The name of the unit that failed to be acknowledged.</param>
    internal sealed record CreateFailed(string Name);

    /// <summary>
    /// Message indicating that a unit has started its completion process.
    /// </summary>
    /// <param name="Name">The name or ID of the unit starting completion.</param>
    internal sealed record StartComplete(string Name);

    /// <summary>
    /// Message indicating that a unit has successfully completed its operation.
    /// </summary>
    /// <param name="Name">The name or ID of the unit that completed successfully.</param>
    internal sealed record CompleteSucceeded(string Name);

    /// <summary>
    /// Message indicating that a unit has failed to complete its operation.
    /// </summary>
    /// <param name="Name">The name or ID of the unit that failed to complete.</param>
    internal sealed record CompleteFailed(string Name);
}
