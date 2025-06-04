using Model = Simulation.Unit.Model;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents a grouped set of producers that are laid out in a regular grid pattern within the simulation.
    /// Each group includes configuration for positioning, spacing, model type and processing alignment.
    /// </summary>
    /// <param name="id">Unique identifier for the group, used to label generated producers.</param>
    /// <param name="model">The type of producer model to instantiate within this group.</param>
    /// <param name="spacing">The spacing in pixels between each producer unit in the group.</param>
    /// <param name="position">The top-left anchor position of the group layout in simulation space.</param>
    /// <param name="processingPos">The relative position for the processing unit associated with each producer.</param>
    /// <param name="Dimension">The total width and height covered by the group layout.</param>
    internal class ProducerGroup(int id, Model model, float spacing, Vector2 position, Vector2 processingPos, Vector2 Dimension)
    {
        /// <summary>
        /// The group ID, typically used to incrementally label each producer created from this group.
        /// </summary>
        internal int Id { get; } = id;

        /// <summary>
        /// The model type of the producers within this group.
        /// </summary>
        internal Model Model { get; } = model;

        /// <summary>
        /// The spacing between each producer unit in the layout grid.
        /// </summary>
        internal float Spacing { get; } = spacing;

        /// <summary>
        /// The starting top-left position for this group in simulation space.
        /// </summary>
        internal Vector2 Position { get; } = position;

        /// <summary>
        /// The processing unit's relative position for each producer in the group.
        /// </summary>
        internal Vector2 ProcessingPos { get; } = processingPos;

        /// <summary>
        /// The total area this group should span, used to determine the number of producers to place.
        /// </summary>
        internal Vector2 Dimension { get; } = Dimension;
    }
}
