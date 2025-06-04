using Interaction = Simulation.Unit.Interaction;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UnitProduction
{
    /// <summary>
    /// Defines the visual and interaction-related specifications of a production unit ("Producer") 
    /// in the simulation. This includes how it interacts with other agents, as well as its default 
    /// display color, processing state color and text label color.
    /// </summary>
    /// <param name="interactions">
    /// A dictionary mapping input interaction types 
    /// to relative 2D positions where these interactions occur on the producer.
    /// </param>
    /// <param name="color">The base color used to render the producer.</param>
    /// <param name="processingColor">The color used to indicate an active or processing state.</param>
    /// <param name="textColor">The color used for drawing text labels on the producer.</param>
    internal class ProducerSpecs(
        Dictionary<Interaction, (uint Ticks, uint Cost)> interactions,
        Vector4 color,
        Vector4 processingColor,
        Vector4 textColor
        )
    {
        /// <summary>
        /// Defines the time and cost associated with each interaction type, relative to the producer's processing capacity.
        /// Ticks represents the time required for the interaction, while Cost represents the associated cost.
        /// Initialized from the producer model specifications.
        /// </summary>
        internal Dictionary<Interaction, (uint Ticks, uint Cost)> Interactions { get; } = interactions;

        /// <summary>
        /// Gets the default base color for rendering the producer.
        /// </summary>
        internal Vector4 Color { get; } = color;

        /// <summary>
        /// Gets the color used to represent the producer processing area.
        /// </summary>
        internal Vector4 ProcessingColor { get; } = processingColor;

        /// <summary>
        /// Gets the color used for any text labels displayed on or around the producer.
        /// </summary>
        internal Vector4 TextColor { get; } = textColor;
    }
}
