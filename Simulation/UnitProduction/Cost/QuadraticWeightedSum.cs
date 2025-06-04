namespace Simulation.UnitProduction.Cost
{
    /// <summary>
    /// Production cost strategy that calculates cost based on queue length and spatial distance.
    /// The X and Y components of the distance are squared and the queue size is weighted more heavily.
    /// </summary>
    internal class QuadraticWeightedSum : ProductionCost
    {
        /// <summary>
        /// Calculates the total cost for a dummy unit, factoring in spatial cost and queue size,
        /// then scaling the result by a transport cost multiplier.
        /// </summary>
        /// <param name="production">The base production cost (time = X and resource = Y).</param>
        /// <param name="queue">The number of units currently queued.</param>
        /// <param name="transportCost">A multiplier representing transport complexity or weight.</param>
        /// <returns>The weighted cost value.</returns>
        internal override ulong CalculateDummy((ulong Ticks, ulong Cost) production, int queue, ulong transportCost)
        {
            var costXY = (production.Ticks * production.Ticks * 2) + (production.Cost * production.Cost);

            var sum = (ulong)(queue * queue) + costXY;
            sum *= transportCost;

            return sum;
        }

        /// <summary>
        /// Calculates the production cost for MQTT-based bidding, excluding transport cost.
        /// Factors in queue size and squared spatial distance.
        /// </summary>
        /// <param name="production">A vector representing spatial cost (typically distance in X and Y).</param>
        /// <param name="queue">The number of units currently queued.</param>
        /// <returns>The weighted cost value.</returns>
        internal override ulong CalculateMQTT((ulong Ticks, ulong Cost) production, int queue)
        {
            var costXY = (production.Ticks * production.Ticks * 2) + (production.Cost * production.Cost);

            var sum = (ulong)(queue * queue) + costXY;

            return sum;
        }

        internal override string GetID()
        {
            return "QuadraticWeightedSum";
        }
    }
}
