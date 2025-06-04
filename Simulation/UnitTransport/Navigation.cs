using Algorithm = Simulation.UnitTransport.Pathfinding.Algorithm;
using Behavior = Simulation.UnitTransport.Steering.Behavior;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Manages navigation logic by combining pathfinding and steering behaviors.
    /// <list type="bullet">
    ///   <item><description><b>Pathfinding:</b> Optionally calculates an initial path to a target.</description></item>
    ///   <item><description><b>Steering:</b> Adjusts agent movement based on real-time behaviors.</description></item>
    ///   <item><description><b>Guidance:</b> Combines both pathfinding and steering for complete navigation control.</description></item>
    /// </list>
    /// </summary>
    /// <param name="steering">
    /// The steering behavior composite used to adjust agent movement in real-time.
    /// </param>
    /// <param name="pathfinding">
    /// The optional pathfinding algorithm used to calculate an initial path to the target. Can be omitted if only steering is needed.
    /// </param>
    internal class Navigation(
        Behavior steering,
        Algorithm? pathfinding = null
        )
    {
        /// <summary>
        /// The steering behavior composite used to adjust agent movement in real-time.
        /// <list type="bullet">
        ///   <item><description><b>Steering:</b> Controls the movement of the agent during navigation, adapting to real-time conditions.</description></item>
        /// </list>
        /// </summary>
        private readonly Behavior _steering = steering;

        /// <summary>
        /// The optional pathfinding algorithm used to calculate an initial path to the target.
        /// <list type="bullet">
        ///   <item><description><b>Pathfinding:</b> Calculates a target path, if provided, for the agent to follow.</description></item>
        ///   <item><description><b>Optional:</b> This behavior can be omitted if direct steering is preferred.</description></item>
        /// </list>
        /// </summary>
        private readonly Algorithm? _pathfinding = pathfinding;

        /// <summary>
        /// Computes the navigation for a given context by first calculating a path (if a pathfinding algorithm is set),
        /// then applying steering behavior to guide the agent along that path or toward the target.
        /// <list type="bullet">
        ///   <item><description><b>Path Calculation:</b> Uses the pathfinding algorithm if available to calculate a path.</description></item>
        ///   <item><description><b>Steering Application:</b> Applies the steering behavior to adjust agent movement based on the calculated path or target.</description></item>
        ///   <item><description><b>Context:</b> Requires a navigation context to operate, which includes the agent state and target information.</description></item>
        /// </list>
        /// </summary>
        /// <param name="context">The current navigation context containing agent state and target information.</param>
        internal void Guidance(NavigationContext context)
        {
            _pathfinding?.FindPath(context);
            _steering.Compute(context);
        }

        /// <summary>
        /// Returns an identifier for this navigation framework.
        /// <list type="bullet">
        ///   <item><description><b>ID:</b> Combines the pathfinding and steering IDs (if available) to create a unique identifier.</description></item>
        ///   <item><description><b>Format:</b> The ID includes the pathfinding ID followed by the steering ID.</description></item>
        /// </list>
        /// </summary>
        internal string GetID()
        {
            var id = _pathfinding != null ? _pathfinding.GetID() + "_" : string.Empty;
            return $"{id}{_steering.GetID()}";
        }
    }
}
