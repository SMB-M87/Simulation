using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Serves as the abstract base for all steering behaviors used to influence agent movement.
    /// Each behavior contributes a weighted force vector to guide navigation.
    /// <list type="bullet">
    ///   <item><description><b>Force:</b> The calculated steering force produced by this behavior, which influences the agent’s movement.</description></item>
    ///   <item><description><b>Compute method:</b> Computes the steering force based on the provided navigation context, which includes agent and environment data.</description></item>
    ///   <item><description><b>GetID method:</b> Optionally returns an identifier for this behavior, useful for debugging or logging purposes.</description></item>
    /// </list>
    /// </summary>
    internal abstract class Behavior()
    {
        /// <summary>
        /// The calculated steering force produced by this behavior.
        /// </summary>
        internal Vector2 Force { get; set; } = Vector2.Zero;

        /// <summary>
        /// Computes the steering force for this behavior using the provided context.
        /// </summary>
        /// <param name="ctx">The current navigation context containing agent and environment data.</param>
        internal abstract void Compute(NavigationContext ctx);

        /// <summary>
        /// Returns an optional identifier for this behavior.
        /// Can be overridden for debugging or logging purposes.
        /// </summary>
        internal virtual string GetID()
        {
            return string.Empty;
        }
    }
}
