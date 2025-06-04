using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behavior that repels an agent from axis-aligned borders using a precise rectangular model.
    /// <list type="bullet">
    ///   <item><description><b>Repulsion Force:</b> Calculates a repulsive force to move the agent away from borders based on proximity.</description></item>
    ///   <item><description><b>Side and Corner Handling:</b> Handles both sides and corners of the rectangular borders for accurate avoidance without relying on circular approximation.</description></item>
    ///   <item><description><b>Prediction:</b> Looks ahead by a few steps to ensure the agent avoids clipping corners and resolves potential collisions smoothly.</description></item>
    ///   <item><description><b>Safe Navigation:</b> Uses a safe distance threshold to ensure the agent maintains a safe distance from obstacles while navigating.</description></item>
    /// </list>
    /// </summary>
    internal class BorderRepulsionRect() : Behavior
    {
        /// <summary>
        /// The repulsion strength applied when the agent approaches a border. 
        /// Higher values result in stronger repulsion forces.
        /// </summary>
        private readonly float _kRepel = 10f;

        /// <summary>
        /// The minimum safe distance between the agent and the border before repulsion forces are applied. 
        /// Prevents the agent from colliding too closely with the border.
        /// </summary>
        private readonly float _safeDistance = 2f;

        /// <summary>
        /// The number of steps ahead the agent looks to predict potential collisions and avoid clipping corners 
        /// during high-speed movement.
        /// </summary>
        private readonly int _lookAhead = 2;

        /// <summary>
        /// Computes a force that repels the rectangular agent from axis-aligned border segments.
        /// The force is calculated both from the sides and the corners of the segment, ensuring
        /// tight but safe navigation around grid-based obstacles without relying on a circular radius.
        /// </summary>
        /// <param name="ctx">The navigation context containing the agent and local environment.</param>
        internal override void Compute(NavigationContext ctx)
        {
            var P0 = ctx.Agent.Center;
            var P = P0 + ctx.Agent.Velocity * _lookAhead;

            var half = ctx.Agent.Dimension / 2f;
            var force = Vector2.Zero;

            foreach (var (A, B) in ctx.Borders)
            {
                if (Math.Abs(A.X - B.X) < float.Epsilon)
                {
                    var xWall = A.X;
                    var yMin = Math.Min(A.Y, B.Y);
                    var yMax = Math.Max(A.Y, B.Y);

                    if (P.Y >= yMin && P.Y <= yMax)
                    {
                        var dx = P.X - xWall;
                        var rawDist = Math.Abs(dx) - half.X;
                        ApplySideRepulsion(ref force, new Vector2(MathF.Sign(dx), 0f), rawDist);
                    }
                }
                else if (Math.Abs(A.Y - B.Y) < float.Epsilon)
                {
                    var yWall = A.Y;
                    var xMin = Math.Min(A.X, B.X);
                    var xMax = Math.Max(A.X, B.X);

                    if (P.X >= xMin && P.X <= xMax)
                    {
                        var dy = P.Y - yWall;
                        var rawDist = Math.Abs(dy) - half.Y;
                        ApplySideRepulsion(ref force, new Vector2(0f, MathF.Sign(dy)), rawDist);
                    }
                }
            }

            foreach (var (A, B) in ctx.Borders)
                foreach (var corner in new[] { A, B })
                {
                    var diff = P - corner;

                    var dx = Math.Max(Math.Abs(diff.X) - half.X, 0f);
                    var dy = Math.Max(Math.Abs(diff.Y) - half.Y, 0f);
                    var d = MathF.Sqrt(dx * dx + dy * dy);

                    if (d < _safeDistance)
                    {
                        var dir = diff.Length() > float.Epsilon
                                  ? diff / diff.Length()
                                  : Vector2.Zero;

                        var mag = d <= 0f
                                  ? _kRepel * 3f
                                  : _kRepel * (1f / (d * d) - 1f / (_safeDistance * _safeDistance));

                        if (mag > 0f)
                            force += dir * mag;
                    }
                }

            Force = force;
        }

        /// <summary>
        /// Adds inverse‑square repulsion for a flat wall side.
        /// </summary>
        private void ApplySideRepulsion(ref Vector2 total, Vector2 normal, float rawDist)
        {
            if (rawDist >= _safeDistance) // outside influence zone
                return;

            var mag = rawDist <= 0f
                        ? _kRepel * 3f
                        : _kRepel * (1f / (rawDist * rawDist) - 1f / (_safeDistance * _safeDistance));

            if (mag > 0f)
                total += normal * mag;
        }
    }
}
