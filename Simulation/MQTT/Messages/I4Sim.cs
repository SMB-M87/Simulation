namespace Simulation.MQTT.Messages
{
    /// <summary>
    /// Contains topic strings used for MQTT communication between the Simulation and the Object Server (OS).
    /// Centralizes topic generation for consistent usage across message handling.
    /// </summary>
    internal static class I4Sim
    {
        /// <summary>
        /// Topic for sending Create messages (Sim → OS).
        /// </summary>
        public const string Create = "i4sim/create";

        /// <summary>
        /// Topic for receiving acknowledgment of Create messages (OS → Sim).
        /// </summary>
        public const string CreateAck = "i4sim/create/ack";

        /// <summary>
        /// Topic for sending Purge messages to clean up the simulation (Sim → OS).
        /// </summary>
        public const string Purge = "i4sim/purge";

        /// <summary>
        /// Returns the topic for sending StateChange messages for a specific unit (Sim → OS).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string StateChange(string name) => $"i4sim/{name}/stateChange";

        /// <summary>
        /// Returns the topic for receiving RequestCost messages for a specific unit (OS → Sim).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string RequestCost(string name) => $"i4sim/{name}/requestCost";

        /// <summary>
        /// Returns the topic for sending ResponseCost messages for a specific unit (Sim → OS).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string ResponseCost(string name) => $"i4sim/{name}/responseCost";
        /// <summary>
        /// Returns the topic for receiving Perform interaction messages for a specific unit (OS → Sim).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string Perform(string name) => $"i4sim/{name}/perform";

        /// <summary>
        /// Returns the topic for receiving acknowledgment of Perform actions (Sim → OS).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string PerformAck(string name) => $"i4sim/{name}/perform/ack";

        /// <summary>
        /// Returns the topic for sending Complete interaction messages (Sim → OS).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string Complete(string name) => $"i4sim/{name}/complete";

        /// <summary>
        /// Returns the topic for receiving acknowledgment of Complete actions (OS → Sim).
        /// </summary>
        /// <param name="name">The name or ID of the simulation unit.</param>
        internal static string CompleteAck(string name) => $"i4sim/{name}/complete/ack";
    }
}
