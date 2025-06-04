using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a ResponseCost message sent by the simulation in reply to a RequestCost message.
    /// Contains the calculated cost of executing a proposed interaction.
    /// </summary>
    internal class ResponseCost : Message
    {
        /// <summary>
        /// The numeric cost value returned by the simulation.
        /// Ignored in JSON if not explicitly set (i.e., default value).
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
        [JsonPropertyName("cost")]
        public ulong Cost { get; set; }

        /// <summary>
        /// Initializes a new ResponseCost message with the given cost value and sets its MessageType.
        /// </summary>
        /// <param name="cost">The cost value associated with the requested interaction.</param>
        public ResponseCost(ulong cost)
        {
            Cost = cost;
            MessageType = MessageType.ResponseCost;
        }
    }
}
