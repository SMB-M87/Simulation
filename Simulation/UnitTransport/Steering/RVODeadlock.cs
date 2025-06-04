using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behaviour that combines two main forces:
    /// <list type="bullet">
    ///   <item>
    ///     <description>
    ///       <c>F1</c> – an attraction force that pulls the agent toward its next waypoint or goal.
    ///     </description>
    ///   </item>
    ///   <item>
    ///     <description>
    ///       <c>F2</c> – a tangent-based repulsion force derived from the Reciprocal Velocity Obstacle (RVO) formulation.
    ///     </description>
    ///   </item>
    /// </list>
    ///
    /// To break positional deadlocks (<c>F1 + F2 ≈ 0</c>), this class:
    /// <list type="bullet">
    ///   <item>
    ///     <description>
    ///       Applies an exponential angular bias (<see cref="_distanceGain"/>) so we prefer passing on the outside.
    ///     </description>
    ///   </item>
    ///   <item>
    ///     <description>
    ///       Uses a spring model (<see cref="_springK"/>) at the tangent point to ensure a residual force.
    ///     </description>
    ///   </item>
    /// </list>
    ///
    /// It also blends in an Arrive behaviour to slow the agent smoothly as it reaches its final destination.
    /// </summary>
    internal class RVODeadlock : Behavior
    {
        /// <summary>
        /// Weight applied to the signed angular difference θ<sub>A</sub>. 
        /// Larger values give a stronger bias to pass on the “outside”.
        /// </summary>
        private readonly float _distanceGain = 0.1f;

        /// <summary>
        /// Spring constant that turns <c>baseOffset − dAway</c> into a Hooke‑like force.
        /// </summary>
        private readonly float _springK = 0.1f;

        /// <summary>Waypoints closer than <c>Dimension · ProximityFactor</c> are considered reached.</summary>
        private readonly float _proximityFactor = 0.45f;

        /// <summary>How many seconds into the future we expand the collision radius by <c>MaxSpeed · LookAhead</c>.</summary>
        private readonly float _lookAHead = 8f;

        /// <summary>
        /// Main entry‑point called once per simulation step.
        /// Computes and stores the steering force that will be integrated by the physics layer.
        /// </summary>
        /// <param name="ctx">Navigation context that holds <see cref="NavigationContext.Agent"/> (self), neighbours, walls, etc.</param>
        internal override void Compute(NavigationContext ctx)
        {
            var agent = ctx.Agent;
            var center = agent.Center;                 // current position ( mᵢ )
            var desired = Vector2.Zero;                // F1 – desired direction towards the current waypoint/goal

            // ───────────────────────────────────────────────────────────────
            // 1. Attraction force F1 : straight line to the current waypoint
            // ───────────────────────────────────────────────────────────────
            if (agent.Destination == Vector2.Zero)
            {
                // No destination – nothing to do for this behaviour.
                Force = desired;
                return;
            }

            var arrival = true;                       // will be set to false if we still have remaining waypoints after this frame
            var dest = agent.Destination;              // final destination, or the head of the path queue if any

            // Use explicit path queue when present
            if (agent.Path.Count > 0)
            {
                dest = agent.Path.Peek();
                var dist = (dest - center).Length();

                // Pop waypoint when we are “close enough”
                while (ctx.Agent.Path.Count > 1 &&
                       dist < ctx.Agent.Dimension.Length() * _proximityFactor)
                {
                    ctx.Agent.Path.Pop();
                    dest = ctx.Agent.Path.Peek();
                    dist = (dest - ctx.Agent.Center).Length();
                }
                arrival = ctx.Agent.Path.Count <= 1;   // still more waypoints? then we are not in Arrive mode yet
            }

            // F1 as a unit vector
            desired = dest - center;
            if (desired.LengthSquared() > 1e-4f)
                desired = Vector2.Normalize(desired);

            // ───────────────────────────────────────────────────────────────
            // 2. Repulsion forces F2 : RVO tangent‑based collision avoidance
            // ───────────────────────────────────────────────────────────────
            var repulsionForce = Vector2.Zero;

            foreach (var neighbor in ctx.Neighbors)
            {
                // Combined radius of the two agents (they are rectangles but we approximate them with circles)
                var safetyRadius = ctx.Agent.Radius + neighbor.Radius;

                // Expand radius by look‑ahead so that slower agents start manoeuvring sooner
                var collisionThreshold = safetyRadius + ctx.Agent.MaxSpeed * _lookAHead;

                // Base offset (Hooke’s reference length)
                var baseOffset = safetyRadius + ctx.Agent.Radius;

                // vector from neighbour to self
                var offset = center - neighbor.Center;
                var dist = offset.Length();

                // Skip when out of range or distance is numerically unsafe
                if (dist >= collisionThreshold || dist <= float.Epsilon)
                    continue;

                // φ : bearing to neighbour, θ : half‑angle of the velocity obstacle
                var phi = MathF.Atan2(offset.Y, offset.X);
                var ratio = Math.Clamp(safetyRadius / dist, -1f, 1f);
                var theta = MathF.Asin(ratio);

                // Arbitrarily choose the “right‑hand” tangent (φ + θ); the symmetric left one would be (φ − θ)
                var phi1 = phi + theta;

                // Tangent point t₁ on neighbour circumference
                var tangent = new Vector2(
                    neighbor.Center.X + safetyRadius * MathF.Cos(phi1),
                    neighbor.Center.Y + safetyRadius * MathF.Sin(phi1)
                );

                // Vector from our centre to the tangent point (points outside the velocity obstacle)
                var away = center - tangent;
                var dAway = away.Length();

                if (dAway <= float.Epsilon)
                    continue; // tangent is numerically too close → ignore

                // unit vector
                var awayDir = away / dAway;

                // Angle between “desired” (F1) and the tangent vector (θA in the slides)
                var vecIn = tangent - center;                   // points to tangent
                var alpha = MathF.Atan2(vecIn.Y, vecIn.X);
                var gamma = MathF.Atan2(desired.Y, desired.X);  // desired heading
                var thetaA = WrapPi(alpha - gamma);             // signed difference normalised to (‑π, π]

                // Exponential angular gain ( K(β) = e^{k·β} )
                var Fp = MathF.Exp(_distanceGain * thetaA);

                // Linear spring model : F = k ( d₀ − d )
                var Fscalar = _springK * (baseOffset - dAway);

                // Full repulsion magnitude combined with angular gain
                var F = Fp * Fscalar;

                // Accumulate vectorially
                repulsionForce += F * awayDir;
            }

            // ───────────────────────────────────────────────────────────────
            // 3. Combine forces F1 + F2
            // ───────────────────────────────────────────────────────────────
            var desiredDir = desired + repulsionForce;

            if (desiredDir.LengthSquared() <= 1e-4f)
            {
                // All forces cancel out → stop (rare but protects against NaNs)
                Force = Vector2.Zero;
                return;
            }
            desiredDir = Vector2.Normalize(desiredDir);

            // ───────────────────────────────────────────────────────────────
            // 4. Arrive behaviour : slow down smoothly when close to the final goal
            // ───────────────────────────────────────────────────────────────
            var distance = (agent.Destination - center).Length();
            var slowRadius = agent.Dimension.Length() * _proximityFactor;
            var speed = arrival && distance < slowRadius
                             ? agent.MaxSpeed * (distance / slowRadius)
                             : agent.MaxSpeed;

            // Steering = desired velocity − current velocity
            var steering = desiredDir * speed - agent.Velocity;

            // Clamp to maximum force for stability
            if (steering.Length() > agent.MaxForce)
                steering = Vector2.Normalize(steering) * agent.MaxForce;

            Force = steering; // final output used by the integrator
        }

        /// <summary>
        /// Helper that wraps <paramref name="angle"/> to the range (‑π, π].
        /// </summary>
        private static float WrapPi(float angle)
        {
            while (angle > MathF.PI) angle -= 2 * MathF.PI;
            while (angle <= -MathF.PI) angle += 2 * MathF.PI;
            return angle;
        }
    }
}
