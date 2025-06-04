namespace Simulation.UnitProduction
{
    /// <summary>
    /// Defines a strategy interface for calculating production costs
    /// based on operation cost, queue size and transport effort.
    /// </summary>
    internal abstract class ProductionCost
    {
        /// <summary>
        /// Calculates the production cost for the dummy bidding procedure.
        /// </summary>
        /// <param name="production">The base production cost (time = Ticks and resource = Cost).</param>
        /// <param name="queue">The current queue length for the operation.</param>
        /// <param name="transportCost">The associated transport cost.</param>
        /// <returns>The total calculated production cost.</returns>
        internal abstract ulong CalculateDummy((ulong Ticks, ulong Cost) production, int queue, ulong transportCost);

        /// <summary>
        /// Calculates the production cost for the MQTT bidding procedure.
        /// </summary>
        /// <param name="production">The base production cost (time and resource).</param>
        /// <param name="queue">The current queue length for the operation.</param>
        /// <returns>The total calculated production cost.</returns>
        internal abstract ulong CalculateMQTT((ulong Ticks, ulong Cost) production, int queue);

        /// <summary>
        /// Gets the string identifier of this cost strategy.
        /// </summary>
        /// <returns>The name of the cost strategy.</returns>
        internal abstract string GetID();
    }
}
