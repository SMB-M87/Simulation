using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Defines a strategy interface for calculating transport costs
    /// based on positional data.
    /// 
    /// This class provides the base structure for all transport cost calculation strategies,
    /// allowing for different implementations to be used based on specific requirements.
    /// <list type="bullet">
    ///   <item><description><b>Strategy Pattern:</b> Serves as an abstract class for different transport cost calculation strategies.</description></item>
    ///   <item><description><b>Positional Data:</b> The calculation is based on the positional data of the transport unit and its destination.</description></item>
    ///   <item><description><b>Extensibility:</b> Allows multiple strategies to be implemented and chosen dynamically depending on simulation needs.</description></item>
    /// </list>
    /// </summary>
    internal abstract class TransportCost
    {
        /// <summary>
        /// Calculates the transport cost between two positions.
        /// <list type="bullet">
        ///   <item><description><b>Position:</b> The starting position of the unit being transported.</description></item>
        ///   <item><description><b>Destination:</b> The target destination the unit needs to reach.</description></item>
        ///   <item><description><b>Cost:</b> Returns an integer value representing the transport cost, which could depend on distance or other factors.</description></item>
        /// </list>
        /// </summary>
        /// <param name="position">The starting position.</param>
        /// <param name="destination">The target destination position.</param>
        /// <returns>The calculated transport cost as an integer.</returns>
        internal abstract ulong Calculate(Vector2 position, Vector2 destination);

        /// <summary>
        /// Returns an identifier for this transport cost.
        /// <list type="bullet">
        ///   <item><description><b>ID:</b> The identifier serves to distinguish different transport cost calculation strategies.</description></item>
        ///   <item><description><b>Usage:</b> Typically used to identify the transport strategy in logs or configuration settings.</description></item>
        /// </list>
        /// </summary>
        /// <returns>A string representing the identifier for this transport cost.</returns>
        internal abstract string GetID();
    }
}
