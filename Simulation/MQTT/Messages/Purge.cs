namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a Purge message sent by the simulation to indicate that it is shutting down
    /// and all associated resources or units should be cleaned up.
    /// </summary>
    internal class Purge : Message
    {
        /// <summary>
        /// Initializes a new Purge message and sets its MessageType.
        /// </summary>
        public Purge()
        {
            MessageType = MessageType.Purge;
        }
    }
}
