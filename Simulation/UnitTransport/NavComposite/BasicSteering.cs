using Simulation.UnitTransport.Steering;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.NavComposite
{
    /// <summary>
    /// A composite steering behavior that combines multiple forces:
    /// <list type="bullet">
    ///   <item><description><b>Border Avoidance:</b> Prevents the agent from colliding with walls or boundaries in the environment.</description></item>
    ///   <item><description><b>Immediate Collision Resolution:</b> Resolves immediate overlaps with other agents by applying a repulsive force.</description></item>
    ///   <item><description><b>Predictive Collision Avoidance:</b> Anticipates future collisions and adjusts the agent’s velocity to avoid them before they occur.</description></item>
    ///   <item><description><b>Seek and Arrival:</b> Directs the agent toward its destination and gradually slows down as it approaches, ensuring smooth arrival.</description></item>
    /// </list>
    /// 
    /// Each sub-behavior is weighted and prioritized based on its relevance in the current context. The computed steering force is the result of combining these forces:
    /// <list type="bullet">
    ///   <item><description><b>_borderWeight:</b> Controls the influence of the border avoidance behavior.</description></item>
    ///   <item><description><b>_collisionWeight:</b> Controls the influence of the immediate collision resolution behavior.</description></item>
    ///   <item><description><b>_predictiveWeight:</b> Controls the influence of the predictive collision avoidance behavior.</description></item>
    ///   <item><description><b>_seekWeight:</b> Controls the influence of the seek and arrival behavior.</description></item>
    /// </list>
    /// </summary>
    internal class BasicSteering() : Behavior
    {
        private readonly BorderRepulsionRect _border = new();
        private readonly CollisionDetection _collision = new();
        private readonly CollisionAvoidance _predictive = new();
        private readonly SeekAndArrival _seek = new();

        private float _borderWeight = 1.0f;
        private float _collisionWeight = 1.0f;
        private float _predictiveWeight = 1.0f;
        private float _seekWeight = 1.0f;

        /// <summary>
        /// Computes the combined steering forces for the agent based on multiple behaviors:
        /// <list type="bullet">
        ///   <item><description><b>_border.Force:</b> Represents the border avoidance force.</description></item>
        ///   <item><description><b>_collision.Force:</b> Represents the immediate collision resolution force.</description></item>
        ///   <item><description><b>_predictive.Force:</b> Represents the predictive collision avoidance force.</description></item>
        ///   <item><description><b>_seek.Force:</b> Represents the seek and arrival force toward the target destination.</description></item>
        /// </list>
        /// 
        /// The forces from each of the behaviors are weighted and combined to determine the agent’s acceleration.
        /// 
        /// <list type="bullet">
        ///   <item><description><b>Input:</b> Receives the <see cref="NavigationContext"/> which includes the agent's state (position, velocity, etc.) and the environment.</description></item>
        ///   <item><description><b>Logic:</b> Each behavior computes a force that is weighted based on its relevance in the current context. The final acceleration is calculated by summing up these weighted forces.</description></item>
        ///   <item><description><b>Output:</b> The agent’s acceleration, velocity, and position are updated based on the computed forces and their weights. The velocity is capped by the agent’s max speed, and the acceleration is capped by the max force. The final position is updated accordingly.</description></item>
        /// </list>
        /// </summary>
        internal override void Compute(NavigationContext ctx)
        {
            _border.Compute(ctx);
            _collision.Compute(ctx);
            _predictive.Compute(ctx);
            _seek.Compute(ctx);
            ComputeWeights();

            ctx.Agent.Acceleration =
                _border.Force * _borderWeight +
                _collision.Force * _collisionWeight +
                _predictive.Force * _predictiveWeight +
                _seek.Force * _seekWeight
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
        /// Computes and adjusts the weights for each steering behavior based on their current forces.
        /// The weights are used to prioritize and combine the different forces in the steering calculation.
        /// <list type="bullet">
        ///   <item><description><b>_borderWeight:</b> The weight of the border avoidance behavior. Set to 1.0 if the border avoidance force is non-zero, otherwise 0.0.</description></item>
        ///   <item><description><b>_collisionWeight:</b> The weight of the collision resolution behavior. Set to 1.0 if the collision force is non-zero, otherwise 0.0.</description></item>
        ///   <item><description><b>_predictiveWeight:</b> The weight of the predictive collision avoidance behavior. Set to 1.0 only if no other forces are applied (i.e., border and collision avoidance forces are zero), otherwise 0.0.</description></item>
        ///   <item><description><b>_seekWeight:</b> The weight of the seek and arrival behavior. Set to 1.0 only if no other forces are applied, otherwise 0.0.</description></item>
        /// </list>
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

            if (
                _predictive.Force != Vector2.Zero &&
                _border.Force == Vector2.Zero &&
                _collision.Force == Vector2.Zero
                )
            {
                _predictiveWeight = 1.0f;
            }
            else
                _predictiveWeight = 0.0f;

            if (
                _seek.Force != Vector2.Zero &&
                _border.Force == Vector2.Zero &&
                _collision.Force == Vector2.Zero &&
                _predictive.Force == Vector2.Zero
                )
                _seekWeight = 1.0f;
            else
                _seekWeight = 0.0f;
        }

        /// <summary>
        /// Returns the identifier for this basic steering behavior.
        /// </summary>
        /// <returns>The string identifier for the behavior.</returns>
        internal override string GetID()
        {
            return "BasicSteering";
        }
    }
}
