using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behavior that reacts to **current physical overlaps** (collisions) between agents.
    /// <list type="bullet">
    ///   <item><description><b>Collision Response:</b> Applies a repulsive force directly away from overlapping neighbors to resolve immediate collisions.</description></item>
    ///   <item><description><b>Reactive Approach:</b> Unlike predictive avoidance, this behavior only reacts to actual overlaps between agents.</description></item>
    ///   <item><description><b>Cooldown Mechanism:</b> Tracks and manages collision counters to handle repeated collision responses and avoid excessive corrections.</description></item>
    /// </list>
    /// </summary>
    internal class CollisionDetection() : Behavior
    {
        /// <summary>
        /// A countdown timer that manages the cooldown period between consecutive collision responses.
        /// <list type="bullet">
        ///   <item><description><b>Cooldown Duration:</b> The timer is used to prevent multiple collision responses from occurring in quick succession.</description></item>
        ///   <item><description><b>Value:</b> Set to 10, representing the number of simulation ticks between collision responses.</description></item>
        /// </list>
        /// </summary>
        private readonly uint _countdown = 10;

        /// <summary>
        /// Computes a repulsive steering force to separate overlapping agents and resolve collisions.
        /// <list type="bullet">
        ///   <item><description><b>Collision Detection:</b> Identifies overlapping agents using axis-aligned bounding box (AABB) collision detection.</description></item>
        ///   <item><description><b>Repulsive Force:</b> Calculates a force pushing the agent away from the overlapping agent based on their relative positions.</description></item>
        ///   <item><description><b>Collision Handling:</b> Tracks the number of collisions and applies a cooldown to prevent excessive collision responses.</description></item>
        ///   <item><description><b>Force Accumulation:</b> Aggregates the repulsive forces from all detected collisions to determine the final steering force.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx">The navigation context, which includes the agent, its current state, and its neighbors.</param>
        internal override void Compute(NavigationContext ctx)
        {
            uint collideCount = 0;
            var totalForce = Vector2.Zero;

            foreach (var other in ctx.Neighbors)
                if (ctx.Agent.IsAABBColliding(other))
                {
                    var direction = ctx.Agent.Center - other.Center;
                    var distance = direction.Length();

                    if (distance > 0.01f)
                    {
                        var repulsionStrength = MathF.Max(1.0f, 10.0f / distance);
                        var force = Vector2.Normalize(direction) * repulsionStrength;

                        totalForce += force;

                        if (other.Collided <= 0)
                        {
                            other.Collided = _countdown - 2;
                        }

                        collideCount++;
                    }
                }
                else
                    totalForce += Vector2.Zero;

            Force = totalForce;

            if (collideCount > 0)
                if (ctx.Agent.Collided == 0)
                {
                    ctx.Agent.Collided = _countdown;
                    Environment.Instance.AddCollision(collideCount);
                }
        }
    }
}
