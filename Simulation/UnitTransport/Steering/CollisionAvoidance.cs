using RectBody = Simulation.Unit.RectBody;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behavior that anticipates future collisions by simulating the future positions of agents
    /// and applying a force to avoid potential collisions within a predictive time window.
    /// <list type="bullet">
    ///   <item><description><b>Collision Prediction:</b> Projects the future positions of agents based on their current velocities.</description></item>
    ///   <item><description><b>Avoidance Force:</b> Applies a steering force to avoid collisions if the projected positions overlap.</description></item>
    ///   <item><description><b>Steps:</b> Simulates future positions by stepping through time over a number of iterations, defined by the <c>_steps</c> field.</description></item>
    ///   <item><description><b>Safety Margin:</b> The <c>_safeMultiplier</c> field increases the collision detection radius to provide a safety margin for avoidance.</description></item>
    /// </list>
    /// </summary>
    internal class CollisionAvoidance() : Behavior
    {
        /// <summary>
        /// Number of steps used to simulate future positions of the agent and its neighbors for collision prediction.
        /// <list type="bullet">
        ///   <item><description><b>_steps:</b> The number of iterations to simulate the agent's future movement and its neighbors' positions. Higher values provide more precise prediction but increase computation time.</description></item>
        /// </list>
        /// </summary>
        private readonly int _steps = 8;

        /// <summary>
        /// Multiplier applied to the dimensions of agents for collision detection to add a safety margin.
        /// <list type="bullet">
        ///   <item><description><b>_safeMultiplier:</b> A factor used to enlarge the detection radius for collision avoidance, ensuring that agents maintain a safe distance.</description></item>
        /// </list>
        /// </summary>
        private readonly float _safeMultiplier = 1.025f;

        /// <summary>
        /// Computes a steering force that anticipates and avoids potential collisions with neighboring agents.
        /// <list type="bullet">
        ///   <item><description><b>Future Position Prediction:</b> Projects agent and neighbor positions over multiple steps to predict collisions.</description></item>
        ///   <item><description><b>Force Calculation:</b> Computes a repulsive force to move the agent away from potential collisions.</description></item>
        ///   <item><description><b>Collision Check:</b> Uses a simple AABB (Axis-Aligned Bounding Box) collision check to detect future overlaps.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx">The context including the agent and its neighbors.</param>
        internal override void Compute(NavigationContext ctx)
        {
            if (ctx.Agent.Velocity == Vector2.Zero)
            {
                Force = Vector2.Zero;
                return;
            }

            var totalForce = Vector2.Zero;
            var count = 0;

            foreach (var other in ctx.Neighbors)
            {
                var force = Vector2.Zero;

                var startDifference = (ctx.Agent.Center - other.Center).Length();
                var combinedDetectionRadius =
                     ctx.Agent.Radius + _steps * ctx.Agent.MaxSpeed +
                     (other.Radius + +_steps * ctx.Agent.MaxSpeed);

                if (startDifference > combinedDetectionRadius)
                    continue;

                for (var i = 0; i <= _steps; i++)
                {
                    var centerPosition = ctx.Agent.Center + ctx.Agent.Velocity * i;
                    var dimension = ctx.Agent.Dimension * _safeMultiplier;
                    var futurePosition = RectBody.GetTopLeftPosition(centerPosition, dimension);

                    for (var o = 0; o <= _steps; o++)
                    {
                        var otherCenterPosition = other.Center + other.Velocity * o;
                        var otherDimension = other.Dimension * _safeMultiplier;
                        var otherFuturePosition = RectBody.GetTopLeftPosition(otherCenterPosition, otherDimension);

                        if (RectBody.IsAABBColliding(
                            futurePosition,
                            dimension,
                            otherFuturePosition,
                            otherDimension))
                        {
                            var diff = centerPosition - otherCenterPosition;
                            var distance = diff.Length();
                            var scale = 1.0f / MathF.Max(distance, 0.01f);
                            scale = scale * scale * scale;
                            force += diff * scale;
                            break;
                        }
                    }
                    if (force != Vector2.Zero)
                    {
                        totalForce += Vector2.Normalize(force);
                        count++;
                        break;
                    }
                }
            }

            if (count > 0)
                totalForce /= count;

            Force = totalForce;
        }
    }
}
