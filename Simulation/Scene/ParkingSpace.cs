using Mover = Simulation.UnitTransport.Mover;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents a single parking space in the simulation.
    /// Each parking space has a unique ID, an optional assigned mover (agent),
    /// and a fixed position in the environment.
    /// </summary>
    internal class ParkingSpace(int id, Mover mover, Vector2 position)
    {
        /// <summary>
        /// Gets the unique identifier of this parking space.
        /// Used to order and reference parking spaces.
        /// </summary>
        internal int ID { get; } = id;

        /// <summary>
        /// Gets or sets the mover (agent) currently assigned to this parking space.
        /// If null, the space is considered available.
        /// </summary>
        internal Mover? Mover { get; set; } = mover;

        /// <summary>
        /// Gets the fixed position of this parking space in world coordinates.
        /// </summary>
        internal Vector2 Position { get; } = position;
    }
}
