using JsonConverter = System.Text.Json.Serialization.JsonConverterAttribute;
using JsonIgnore = System.Text.Json.Serialization.JsonIgnoreAttribute;
using JsonIgnoreCondition = System.Text.Json.Serialization.JsonIgnoreCondition;
using JsonPropertyName = System.Text.Json.Serialization.JsonPropertyNameAttribute;
using State = Simulation.Unit.State;

namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a StateChange message sent from the simulation to the Object Server (OS).
    /// Used to toggle or communicate the current operational state of a simulation unit (e.g., bidding enabled/disabled).
    /// </summary>
    internal class StateChange : Message
    {
        /// <summary>
        /// The name or identifier of the simulation unit whose state is changing.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
        [JsonPropertyName("name")]
        public string Name { get; set; }

        /// <summary>
        /// The new state of the unit, typically represented by the <see cref="State"/> enum.
        /// Ignored in JSON if null.
        /// </summary>
        [JsonConverter(typeof(EnumMemberJsonStringEnumConverter<State>))]
        [JsonPropertyName("state")]
        public State State { get; set; }

        /// <summary>
        /// Initializes a new StateChange message with the specified unit name and new state,
        /// and sets its MessageType accordingly.
        /// </summary>
        /// <param name="name">The name of the unit whose state is changing.</param>
        /// <param name="state">The new state to assign to the unit.</param>
        public StateChange(string name, State state)
        {
            Name = name;
            State = state;
            MessageType = MessageType.StateChange;
        }
    }
}
