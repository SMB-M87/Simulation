using Model = Simulation.Unit.Model;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents a grouped layout of transport units (movers) to be auto-instantiated in a grid formation
    /// within the simulation space.
    /// 
    /// The group defines the mover model, spacing between units, origin position and total area covered.
    /// </summary>
    /// <param name="id">The initial ID for the first mover in the group (subsequent movers increment this).</param>
    /// <param name="model">The mover model to assign to each unit in the group.</param>
    /// <param name="spacing">The spacing between each mover (both horizontal and vertical).</param>
    /// <param name="position">The top-left position where the group layout begins in simulation space.</param>
    /// <param name="Dimension">The total area the group occupies, used to calculate how many movers to create.</param>
    internal class MoverGroup(int id, Model model, float spacing, Vector2 position, Vector2 Dimension)
    {
        /// <summary>
        /// Unique base ID for the group; used when generating mover identifiers.
        /// </summary>
        internal int Id { get; } = id;

        /// <summary>
        /// The model type used for all movers within this group.
        /// </summary>
        internal Model Model { get; } = model;

        /// <summary>
        /// The distance between adjacent movers in the grid layout.
        /// </summary>
        internal float Spacing { get; } = spacing;

        /// <summary>
        /// The top-left anchor position of the group in simulation coordinates.
        /// </summary>
        internal Vector2 Position { get; } = position;

        /// <summary>
        /// The total width and height (in simulation units) that the group will occupy.
        /// </summary>
        internal Vector2 Dimension { get; } = Dimension;
    }
}
