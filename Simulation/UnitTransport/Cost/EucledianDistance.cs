using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Cost
{
    /// <summary>
    /// Implements a transport cost calculation using the Euclidean distance formula.
    /// Suitable for continuous or diagonal movement in 2D space.
    /// </summary>
    internal class EucledianDistance : TransportCost
    {
        /// <summary>
        /// Calculates the straight-line (Euclidean) distance between two points.
        /// </summary>
        /// <param name="position">The starting position.</param>
        /// <param name="destination">The destination position.</param>
        /// <returns>The distance as an unsigned long, truncating any fractional part.</returns>
        internal override ulong Calculate(Vector2 position, Vector2 destination)
        {
            var dx = position.X - destination.X;
            var dy = position.Y - destination.Y;
            var dist = Math.Sqrt(dx * dx + dy * dy);

            return (ulong)(dist);
        }

        /// <summary>
        /// Returns a unique identifier for the pathfinding algorithm being used.
        /// <list type="bullet">
        ///   <item><description><b>Logic:</b> This method returns the name of the algorithm as a string, which in this case is "EucledianDistance".</description></item>
        ///   <item><description><b>Output:</b> A string representing the ID of the algorithm.</description></item>
        /// </list>
        /// </summary>
        internal override string GetID()
        {
            return "EucledianDistance";
        }
    }
}
