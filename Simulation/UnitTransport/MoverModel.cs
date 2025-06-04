using Model = Simulation.Unit.Model;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Provides a predefined mapping between <see cref="Model"/> identifiers and their corresponding 
    /// <see cref="MoverSpecs"/> specifications. 
    /// <list type="bullet">
    ///   <item><description><b>Mapping:</b> Each <see cref="Model"/> is mapped to its associated <see cref="MoverSpecs"/>.</description></item>
    ///   <item><description><b>Payload capacity:</b> Defines the maximum weight the mover can carry (in kilograms).</description></item>
    ///   <item><description><b>Dimensions:</b> Specifies the mover's physical size (width and height) as a <see cref="Vector2"/>.</description></item>
    ///   <item><description><b>Usage:</b> Allows retrieval of a mover's technical specifications based on its model type.</description></item>
    /// </list>
    /// </summary>
    internal static class MoverModel
    {
        /// <summary>
        /// A dictionary mapping each <see cref="Model"/> to its associated <see cref="MoverSpecs"/>, 
        /// including payload capacity (in kilograms) and dimensions (in millimeters).
        /// <remarks>
        /// The dictionary provides a predefined specification for each <see cref="Model"/> in the simulation.
        /// Each model entry includes the payload capacity and physical dimensions relevant to the simulation.
        /// </remarks>
        /// <list type="bullet">
        ///   <item><description><b>Key:</b> The <see cref="Model"/> enum representing a specific mover model.</description></item>
        ///   <item><description><b>Value:</b> The <see cref="MoverSpecs"/> that contains the payload and dimension of the mover.</description></item>
        ///   <item><description><b>Payload:</b> Represents the maximum payload capacity in kilograms.</description></item>
        ///   <item><description><b>Dimensions:</b> Represents the width and height of the mover in millimeters as a <see cref="Vector2"/>.</description></item>
        /// </list>
        /// </summary>
        public static readonly Dictionary<Model, MoverSpecs> Specs = new()
        {
            [Model.APM4220] = new MoverSpecs(0.6f, new(113, 113)),
            [Model.APM4221] = new MoverSpecs(1.0f, new(127, 127)),
            [Model.APM4230] = new MoverSpecs(0.8f, new(115, 155)),
            [Model.APM4330] = new MoverSpecs(1.8f, new(155, 155)),
            [Model.APM4331] = new MoverSpecs(1.2f, new(150, 150)),
            [Model.APM4350] = new MoverSpecs(3.0f, new(155, 235)),
            [Model.APM4550] = new MoverSpecs(4.5f, new(235, 235))
        };
    }
}
