using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a Perform message sent from the Object Server (OS) to the simulation.
    /// Instructs a simulation unit to begin executing an interaction, typically with a specified destination.
    /// </summary>
    internal class Perform : Message
    {
        /// <summary>
        /// The interaction element that specifies the action or service 
        /// to be performed by the receiving unit. 
        /// Required for defining the operation.
        /// </summary>
        [JsonPropertyName("interactionElement")]
        public required string InteractionElement { get; set; }

        /// <summary>
        /// The target destination for the interaction, if applicable.
        /// May be null depending on the interaction type.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("destination")]
        public string? Destination { get; set; }

        /// <summary>
        /// Initializes a new Perform message and sets its MessageType.
        /// </summary>
        public Perform()
        {
            MessageType = MessageType.Perform;
        }
    }
}
