namespace Simulation.UnitProduction.Cost
{
    /// <summary>
    /// Production cost strategy using a linear weighted combination of time, interaction intensity and queue size.
    /// Each component is scaled independently to reflect its impact on production priority or bidding.
    /// </summary>
    internal class LinearWeighted : ProductionCost
    {
        /// <summary>
        /// Weight applied to production time (cost.X).
        /// </summary>
        private readonly double _wTime = 1.0;

        /// <summary>
        /// Weight applied to interaction or resource intensity (cost.Y).
        /// </summary>
        private readonly double _wInteraction = 2.0;

        /// <summary>
        /// Weight applied to the current production queue length.
        /// </summary>
        private readonly double _wQueue = 5.0;

        /// <summary>
        /// Calculates the full production cost for a dummy unit, factoring in time, interaction and queue size,
        /// then scaling the result with a transport multiplier.
        /// </summary>
        /// <param name="production">The base production cost, where X = time and Y = interaction/resource intensity.</param>
        /// <param name="queue">The number of units currently queued.</param>
        /// <param name="transportCost">A scaling factor representing transport difficulty or expense.</param>
        /// <returns>The total weighted production cost.</returns>
        internal override ulong CalculateDummy((ulong Ticks, ulong Cost) production, int queue, ulong transportCost)
        {
            var raw = _wTime * production.Ticks
                    + _wInteraction * production.Cost
                    + _wQueue * queue;

            return (ulong)(raw * transportCost);
        }

        /// <summary>
        /// Calculates the weighted cost for MQTT bidding, excluding transport scaling.
        /// </summary>
        /// <param name="production">The base production cost, where X = time and Y = interaction/resource intensity.</param>
        /// <param name="queue">The number of units currently queued.</param>
        /// <returns>The weighted production cost used for bidding.</returns>
        internal override ulong CalculateMQTT((ulong Ticks, ulong Cost) production, int queue)
        {
            var raw = _wTime * production.Ticks
                    + _wInteraction * production.Cost
                    + _wQueue * queue;

            return (ulong)raw;
        }

        internal override string GetID()
        {
            return "LinearWeighted";
        }
    }
}
