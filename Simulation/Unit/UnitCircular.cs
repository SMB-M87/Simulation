using Akka.Actor;
using MQTTClient = Simulation.MQTT.Client;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.Unit
{
    /// <summary>
    /// Base class representing a circular simulation unit with a unique ID, model type, and operational state.
    /// Provides core functionality such as state toggling and integration with logging and MQTT messaging.
    /// </summary>
    internal class UnitCircular(string id, Model model, Vector2 position, Vector4 color, float radius = 0.0f) : CircularBody(position, color, radius)
    {
        /// <summary>
        /// Gets the unique string ID of the unit (format: Model_ID).
        /// </summary>
        internal string ID { get; } = id;

        /// <summary>
        /// Gets the model associated with this unit.
        /// </summary>
        internal Model Model { get; } = model;

        /// <summary>
        /// Gets or sets the current operational state of the unit.
        /// </summary>
        internal State State { get; set; } = State.Alive;

        /// <summary>
        /// Actor reference to the service requester currently being processed. Defaults to <see cref="ActorRefs.Nobody"/>.
        /// </summary>
        internal IActorRef ServiceRequester { get; set; } = ActorRefs.Nobody;

        /// <summary>
        /// Toggles the current state between <see cref="State.Alive"/> and <see cref="State.Blocked"/>.
        /// Logs the state change and optionally publishes it via MQTT if bidding is enabled.
        /// </summary>
        internal void ToggleState()
        {
            if (State == State.Alive)
                State = State.Blocked;
            else
                State = State.Alive;

            if (UI.Instance.SettingPanel.MQTT.Active)
                MQTTClient.Instance.PublishStateChange(ID, State);
        }
    }
}