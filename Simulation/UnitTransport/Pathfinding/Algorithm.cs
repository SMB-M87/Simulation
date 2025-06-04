using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Pathfinding
{
    /// <summary>
    /// Defines the base class for pathfinding algorithms, which compute a path through
    /// the environment grid based on the agent's goal.
    /// <list type="bullet">
    ///   <item><description><b>Path:</b> The computed path to the destination as a stack of waypoints.
    ///     The top of the stack represents the next position the agent should move toward.</description></item>
    ///   <item><description><b>FindPath:</b> Abstract method to be implemented by derived classes, 
    ///     which finds the path to the destination for the agent using the current navigation context.</description></item>
    ///   <item><description><b>GetID:</b> Returns an optional identifier for this algorithm, 
    ///     which can be overridden for debugging or logging purposes.</description></item>
    /// </list>
    /// </summary>
    internal abstract class Algorithm
    {
        /// <summary>
        /// The computed path to the destination as a stack of waypoints.
        /// The top of the stack represents the next position the agent should move toward.
        /// </summary>
        internal Stack<Vector2> Path { get; set; } = [];

        /// <summary>
        /// Finds a path to the destination for the agent using the current navigation context.
        /// <list type="bullet">
        ///   <item><description><b>Inputs:</b></description></item>
        ///         <list type="bullet">
        ///            <item><description><b>agent:</b> The movable body (agent) whose path is being computed. This includes the agent's current position, velocity, and other relevant state information.</description></item>
        ///            <item><description><b>environment:</b> The navigable grid representing the environment, which includes the terrain, obstacles, and available paths.</description></item>
        ///            <item><description><b>Logic:</b> This method uses the provided agent and environment to compute a path through the grid, considering any obstacles or constraints in the environment.</description></item>
        ///            <item><description><b>Output:</b> The computed path to the destination, which is stored in the <see cref="Path"/> property as a stack of waypoints, with the top of the stack representing the next position the agent should move toward.</description></item>
        ///         </list>
        /// </list>
        /// </summary>
        /// <param name="ctx">The current navigation context containing agent and environment data.</param>
        internal abstract void FindPath(NavigationContext ctx);

        /// <summary>
        /// Returns an optional identifier for this algorithm.
        /// Can be overridden for debugging or logging purposes.
        /// </summary>
        internal virtual string GetID()
        {
            return string.Empty;
        }
    }
}
