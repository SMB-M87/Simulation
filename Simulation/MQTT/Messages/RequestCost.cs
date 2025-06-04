using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a RequestCost message sent from the Object Server (OS) to the simulation.
    /// It asks a unit to provide cost information before sending a Call for Proposal.
    /// </summary>
    internal class RequestCost : Message
    {
        /// <summary>
        /// A string identifier for the specific ServiceRequester, typically used for tracking or responding.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("serviceRequester")]
        public required string ServiceRequester { get; set; }

        /// <summary>
        /// The interaction element associated with the service request, 
        /// typically specifying the type of service or action being requested.
        /// Required for processing the request.
        /// </summary>
        [JsonPropertyName("interactionElement")]
        public required string InteractionElement { get; set; }

        /// <summary>
        /// The optional destination identifier related to the service request, 
        /// representing the target location or endpoint, if applicable.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonPropertyName("destination")]
        public string? Destination { get; set; }

        /// <summary>
        /// Initializes a new RequestCost message and sets its MessageType.
        /// </summary>
        public RequestCost()
        {
            MessageType = MessageType.RequestCost;
        }
    }
}
