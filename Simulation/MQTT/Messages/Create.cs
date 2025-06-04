using JsonConverter = System.Text.Json.Serialization.JsonConverterAttribute;
using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;
using State = Simulation.Unit.State;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a Create message used to register a simulation unit with the Object Server (OS).
    /// Contains identification and capability information needed to instantiate the unit.
    /// </summary>
    internal class Create : Message
    {
        /// <summary>
        /// The unique name of the simulation unit.
        /// Used as both the unit identifier and its location.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("name")]
        public string Name { get; set; }

        /// <summary>
        /// The location of the simulation unit, often the same as its name.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("location")]
        public string Location { get; set; }

        /// <summary>
        /// The model or type name of the simulation unit.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("model")]
        public string Model { get; set; }

        /// <summary>
        /// A list of supported interaction element URLs for this unit.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("interactionElements")]
        public List<string> InteractionElements { get; set; }

        /// <summary>
        /// The initial state of the unit, typically represented by the <see cref="State"/> enum.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonConverter(typeof(EnumMemberJsonStringEnumConverter<State>))]
        [JsonPropertyName("state")]
        public State State { get; set; }

        /// <summary>
        /// Initializes a new Create message with the specified name, model and interaction list.
        /// </summary>
        /// <param name="name">The name of the unit.</param>
        /// <param name="location">The location of the unit.</param>
        /// <param name="model">The model identifier for the unit.</param>
        /// <param name="interactions">A list of supported interaction URLs.</param>
        /// <param name="state">The initial state of the unit.</param>
        public Create(
            string name,
            string location,
            string model,
            List<string> interactions,
            State state
            )
        {
            Name = name;
            Location = location;
            Model = model;
            InteractionElements = interactions;
            State = state;
            MessageType = MessageType.Create;
        }
    }
}
