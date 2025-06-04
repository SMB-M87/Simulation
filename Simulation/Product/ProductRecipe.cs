using Interaction = Simulation.Unit.Interaction;

namespace Simulation.Product
{
    /// <summary>
    /// Contains predefined production recipes for each <see cref="Product"/> type.
    /// Each recipe specifies the exact sequence of interactions needed to manufacture the product.
    /// </summary>
    internal static class ProductRecipe
    {
        /// <summary>
        /// Maps each product to its associated production recipe.
        /// </summary>
        public static readonly Dictionary<Product, ProductSpecs> Specs = new()
        {
            [Product.Trimmer] = new ProductSpecs(
            [
                Interaction.PlaceHousing,
                Interaction.PlaceTrimmerElement,
                Interaction.PlaceLever,
                Interaction.RemoveAssy
            ]),
            [Product.TrimmerPersonalized] = new ProductSpecs(
            [
                Interaction.PlaceHousing,
                Interaction.PlaceTrimmerElement,
                Interaction.PlaceLever,
                Interaction.PlaceCard,
                Interaction.PersonalizeCard,
                Interaction.RemoveAssy
            ])
        };
    }
}
