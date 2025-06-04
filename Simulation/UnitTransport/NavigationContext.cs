using NavigableGrid = Simulation.Scene.NavigableGrid;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Represents the contextual information required by pathfinding and steering behaviors to compute movement decisions
    /// for a single agent within the simulation.
    /// <list type="bullet">
    ///   <item><description><b>Pathfinding Context:</b> Stores necessary information for calculating movement decisions for an agent.</description></item>
    ///   <item><description><b>Influences:</b> Takes into account nearby agents, borders, and the navigable environment.</description></item>
    ///   <item><description><b>Agent Specific:</b> Context is specific to each agent and helps guide its movement.</description></item>
    /// </list>
    /// </summary>
    internal class NavigationContext
    {
        /// <summary>
        /// The agent (movable body) for which navigation is being computed.
        /// <list type="bullet">
        ///   <item><description><b>Agent:</b> The unit (movable body) that the navigation calculations apply to.</description></item>
        ///   <item><description><b>Role:</b> The agent's position and movement are influenced by this context.</description></item>
        /// </list>
        /// </summary>
        internal MovableBody Agent { get; set; } = null!;

        /// <summary>
        /// The grid defining the navigable environment, including cellsize and dimensions.
        /// <list type="bullet">
        ///   <item><description><b>Environment:</b> The grid provides a map of the area where the agent can move.</description></item>
        /// </list>
        /// </summary>
        internal NavigableGrid Environment { get; set; } = null!;

        /// <summary>
        /// The list of nearby agents that may influence the current agent's navigation.
        /// <list type="bullet">
        ///   <item><description><b>Neighbors:</b> A list of other agents whose positions and movements affect the current agent's decision-making.</description></item>
        ///   <item><description><b>Influence:</b> Nearby agents can cause the current agent to adjust its pathfinding or steering behavior.</description></item>
        /// </list>
        /// </summary>
        internal List<MovableBody> Neighbors { get; set; } = [];

        /// <summary>
        /// The list of nearby borders that may influence the current agent's navigation.
        /// <list type="bullet">
        ///   <item><description><b>Borders:</b> Represents virtual barriers that the agent must avoid or navigate around.</description></item>
        ///   <item><description><b>Impact:</b> Borders affect the agent's movement and routing decisions during navigation.</description></item>
        /// </list>
        /// </summary>
        internal List<(Vector2 A, Vector2 B)> Borders { get; set; } = [];
    }
}
