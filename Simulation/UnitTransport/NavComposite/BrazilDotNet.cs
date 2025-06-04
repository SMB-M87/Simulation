using Simulation.UnitTransport.Steering;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.NavComposite
{
    /// <summary>
    /// A lightweight interpretation of the implementation of Brazil de Vries Reciprocal Velocity Obstacles (RVO) for real-time multi-agent systems.
    /// This behavior combines multiple steering strategies to handle obstacles and predict future collisions:
    /// <list type="bullet">
    ///   <item><description><b>Border avoidance:</b> Handles repulsion from environmental borders to prevent agents from colliding with static obstacles.</description></item>
    ///   <item><description><b>Collision detection:</b> Detects collisions between agents and computes a repulsive force to avoid overlapping with other agents.</description></item>
    ///   <item><description><b>RVO-based predictive collision handling:</b> Predicts potential collisions and adjusts movement to prevent deadlocks or overlapping in the future.</description></item>
    /// </list>
    /// </summary>
    internal class BrazilDotNet() : Behavior
    {
        private readonly BorderRepulsionRadius _border = new();
        private readonly CollisionDetection _collision = new();
        private readonly RVODeadlock _predictive = new();

        /// <summary>
        /// The weights that influence the relative importance of each behavior in the final steering force calculation.
        /// <list type="bullet">
        ///   <item><description><b>_borderWeight:</b> Controls the influence of the border avoidance behavior. A higher value increases the influence of border avoidance in the steering calculation.</description></item>
        ///   <item><description><b>_collisionWeight:</b> Controls the influence of the collision detection behavior. A higher value increases the influence of collision avoidance in the steering calculation.</description></item>
        ///   <item><description><b>_predictiveWeight:</b> Controls the influence of the predictive collision handling behavior (RVO-based). A higher value increases the influence of predictive handling in the steering calculation.</description></item>
        /// </list>
        /// </summary>
        private float _borderWeight = 1.0f;
        private float _collisionWeight = 1.0f;
        private float _predictiveWeight = 1.0f;

        /// <summary>
        /// Computes the steering behavior by combining forces from border avoidance, collision detection, and predictive handling.
        /// Updates the agent's acceleration, velocity, and position based on the weighted contribution of each behavior.
        /// </summary>
        /// <param name="ctx">The navigation context containing the agent, environment, and other data necessary to compute movement.</param>
        internal override void Compute(NavigationContext ctx)
        {
            _border.Compute(ctx);
            _collision.Compute(ctx);
            _predictive.Compute(ctx);
            ComputeWeights();

            ctx.Agent.Acceleration =
                _border.Force * _borderWeight +
                _collision.Force * _collisionWeight +
                _predictive.Force * _predictiveWeight
                ;

            if (ctx.Agent.Acceleration != Vector2.Zero && ctx.Agent.Acceleration.Length() > ctx.Agent.MaxForce)
                ctx.Agent.Acceleration = Vector2.Normalize(ctx.Agent.Acceleration) * ctx.Agent.MaxForce;

            if (ctx.Agent.Acceleration != Vector2.Zero)
                ctx.Agent.Velocity += ctx.Agent.Acceleration;

            if (ctx.Agent.Velocity.Length() > ctx.Agent.MaxSpeed)
                ctx.Agent.Velocity = Vector2.Normalize(ctx.Agent.Velocity) * ctx.Agent.MaxSpeed;

            if (ctx.Agent.Velocity != Vector2.Zero)
                ctx.Agent.Position += ctx.Agent.Velocity;
        }

        /// <summary>
        /// Computes the weights for each behavior based on whether their respective forces are non-zero.
        /// Adjusts the relative importance of each behavior (border, collision, and predictive) during the steering calculation.
        /// </summary>
        private void ComputeWeights()
        {
            if (_border.Force != Vector2.Zero)
                _borderWeight = 1.0f;
            else
                _borderWeight = 0.0f;

            if (_collision.Force != Vector2.Zero)
                _collisionWeight = 1.0f;
            else
                _collisionWeight = 0.0f;

            if (_predictive.Force != Vector2.Zero)
            {
                _predictiveWeight = 1.0f;
            }
            else
                _predictiveWeight = 0.0f;
        }

        /// <summary>
        /// Returns the unique identifier for this steering behavior.
        /// </summary>
        /// <returns>The string "BrazilDotNet" representing this behavior's ID.</returns>
        internal override string GetID()
        {
            return "BrazilDotNet";
        }
    }
}
