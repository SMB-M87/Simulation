using Color = Simulation.Util.Color;
using Interaction = Simulation.Unit.Interaction;
using Model = Simulation.Unit.Model;

namespace Simulation.UnitProduction
{
    /// <summary>
    /// Provides a mapping of production unit models to their corresponding <see cref="ProducerSpecs"/> definitions.
    /// 
    /// Each producer model defines interaction points (based on specific <see cref="Interaction"/> types) and visual styles,
    /// including default color, active processing color and label text color.
    /// This setup allows the simulation to render and handle behavior for various production units consistently.
    /// </summary>
    internal static class ProducerModel
    {
        /// <summary>
        /// Dictionary containing specifications for each <see cref="Model"/> of producer.
        /// Each entry defines interaction positions and visual styles for rendering and behavior control.
        /// </summary>
        public static readonly Dictionary<Model, ProducerSpecs> Specs = new()
        {
            [Model.Kuka] =
            new ProducerSpecs(
                new Dictionary<Interaction, (uint Ticks, uint Cost)>
                {
                    [Interaction.PlaceHousing] = new(1, 1),
                    [Interaction.PlaceTrimmerElement] = new(1, 1)
                },
                Color.OrangeDark,
                Color.OrangeDark45,
                Color.Black
            ),

            [Model.Staubli] =
            new ProducerSpecs(
                new Dictionary<Interaction, (uint Ticks, uint Cost)>
                {
                    [Interaction.PlaceLever] = new(1, 1),
                    [Interaction.PlaceCard] = new(1, 1)
                },
                Color.YellowGreen,
                Color.YellowGreen45,
                Color.Black
            ),

            [Model.Viper] =
            new ProducerSpecs(
                new Dictionary<Interaction, (uint Ticks, uint Cost)>
                {
                    [Interaction.PersonalizeCard] = new(1, 1)
                },
                Color.White,
                Color.White45,
                Color.Black
            ),

            [Model.Manuel] =
            new ProducerSpecs(
                new Dictionary<Interaction, (uint Ticks, uint Cost)>
                {
                    [Interaction.PlaceLever] = new(1, 1),
                    [Interaction.RemoveAssy] = new(1, 1)
                },
                Color.Blueatre,
                Color.Blueatre45,
                Color.Black
            )
        };
    }
}
