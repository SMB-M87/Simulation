using JsonConverter = System.Text.Json.Serialization.JsonConverterAttribute;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Base class for all MQTT message types in the simulation.
    /// Contains the shared <see cref="MessageType"/> property used to identify the kind of message being sent or received.
    /// </summary>
    internal class Message
    {
        /// <summary>
        /// The type of MQTT message, serialized as a string using the <see cref="MessageType"/> enum.
        /// Determines how the message should be interpreted by the receiver.
        /// </summary>
        [JsonPropertyName("messageType")]
        [JsonConverter(typeof(EnumMemberJsonStringEnumConverter<MessageType>))]
        public MessageType MessageType { get; set; }
    }
}
