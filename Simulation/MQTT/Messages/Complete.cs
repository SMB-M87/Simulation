namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Represents a message indicating that an interaction has been completed by the simulation.
    /// Sent from Sim → OS as part of the interaction lifecycle.
    /// </summary>
    internal class Complete : Message
    {
        /// <summary>
        /// Initializes a new Complete message and sets its MessageType.
        /// </summary>
        public Complete()
        {
            MessageType = MessageType.Complete;
        }
    }
}
