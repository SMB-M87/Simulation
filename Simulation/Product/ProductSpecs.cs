using Interaction = Simulation.Unit.Interaction;

namespace Simulation.Product
{
    /// <summary>
    /// Defines the production steps (recipe) required to create a specific product.
    /// Each step corresponds to an <see cref="Interaction"/> performed by a producer.
    /// </summary>
    /// <param name="recipe">The ordered list of interactions needed to assemble the product.</param>
    internal class ProductSpecs(List<Interaction> recipe)
    {
        /// <summary>
        /// The sequence of production interactions required to complete this product.
        /// </summary>
        internal List<Interaction> Recipe { get; } = recipe;
    }
}
