using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Cost
{
    /// <summary>
    /// Implements a transport cost calculation using the Manhattan distance heuristic.
    /// Assumes a grid-based environment where movement is axis-aligned (no diagonals).
    /// </summary>
    internal class ManhattanDistance : TransportCost
    {
        /// <summary>
        /// Calculates the Manhattan distance between the given position and destination.
        /// This is the sum of the absolute horizontal and vertical differences.
        /// </summary>
        /// <param name="position">The current position of the unit.</param>
        /// <param name="destination">The target position.</param>
        /// <returns>The Manhattan distance as an integer.</returns>
        internal override ulong Calculate(Vector2 position, Vector2 destination)
        {
            var cost = Math.Abs(position.X - destination.X);
            cost += Math.Abs(position.Y - destination.Y);

            return (ulong)cost;
        }

        /// <summary>
        /// Returns a unique identifier for the pathfinding algorithm being used.
        /// <list type="bullet">
        ///   <item><description><b>Logic:</b> This method returns the name of the algorithm as a string, which in this case is "ManhattanDistance".</description></item>
        ///   <item><description><b>Output:</b> A string representing the ID of the algorithm.</description></item>
        /// </list>
        /// </summary>
        internal override string GetID()
        {
            return "ManhattanDistance";
        }
    }
}
