namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Enumerates the types of MQTT messages exchanged between the Simulation and the Object Server (OS).
    /// Each value represents a specific communication intent or phase within the interaction protocol.
    /// </summary>
    internal enum MessageType
    {
        /// <summary>
        /// Sim -> OS, to create the simulation unit.
        /// </summary>
        Create,

        /// <summary>
        /// After succesfull Create, Perform or Complete.
        /// </summary>
        Acknowledge,

        /// <summary>
        /// Sim -> OS, to switch bidding on/off
        /// </summary>
        StateChange,

        /// <summary>
        /// OS -> Sim, before a Call for Proposal is being sent.
        /// </summary>
        RequestCost,

        /// <summary>
        /// Sim -> OS, reaction on the cost request.
        /// </summary>
        ResponseCost,

        /// <summary>
        /// OS -> Sim, to start the interaction in the simulation.
        /// </summary>
        Perform,

        /// <summary>
        /// Sim -> OS, to acknowledge that an interaction has been performed.
        /// </summary>
        Complete,

        /// <summary>
        /// Sim -> OS, to acknowledge that the simulation is being closed.
        /// </summary>
        Purge
    }
}
