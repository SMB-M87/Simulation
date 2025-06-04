using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents an acknowledgment message sent after a successful Create, Perform, or Complete interaction.
    /// </summary>
    internal class Acknowledge : Message
    {
        /// <summary>
        /// The name of the simulation unit being acknowledged.
        /// This property is ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("name")]
        public string Name { get; set; } = null!;

        /// <summary>
        /// Default constructor for serialization.
        /// </summary>
        public Acknowledge() { }

        /// <summary>
        /// Creates an Acknowledge message with the specified model name and sets the message type.
        /// </summary>
        /// <param name="model">The name or identifier of the acknowledged unit.</param>
        public Acknowledge(string model)
        {
            MessageType = MessageType.Acknowledge;
            Name = model;
        }
    }
}
