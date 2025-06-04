using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Defines the specifications of a transport unit (Mover), including its payload capacity 
    /// and physical dimensions. Used to describe physical constraints and capabilities 
    /// relevant to movement and placement within the simulation.
    /// </summary>
    /// <param name="payload">
    /// The maximum payload capacity in kilograms that the mover can carry.
    /// </param>
    /// <param name="dimension">
    /// The width and height of the mover as a 2D vector.
    /// </param>
    internal class MoverSpecs(float payload, Vector2 dimension)
    {
        /// <summary>
        /// Gets the maximum payload capacity in kilograms that the mover can carry.
        /// </summary>
        internal float PayloadKg { get; } = payload;

        /// <summary>
        /// Gets the dimensions (width and height) of the mover.
        /// </summary>
        internal Vector2 Dimension { get; } = dimension;
    }
}
