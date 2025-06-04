using RectBody = Simulation.Unit.RectBody;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behavior that applies a circular approximation of boundary repulsion.
    /// <list type="bullet">
    ///   <item><description><b>Inverse-square law:</b> Applies a repulsive force when the agent’s circular hull gets closer than <see cref="_safeDistance"/> to a wall segment.</description></item>
    ///   <item><description><b>Force magnitude:</b> The closer the agent is to the wall, the stronger the repulsion. If the agent penetrates the wall, a strong constant force is applied.</description></item>
    ///   <item><description><b>Safe distance:</b> Defines the threshold distance at which the repulsion force starts to activate, preventing collisions by maintaining clearance.</description></item>
    ///   <item><description><b>Repulsion direction:</b> The repulsive force is applied directly away from the wall, keeping the agent away from obstacles.</description></item>
    /// </list>
    /// </summary>
    internal class BorderRepulsionRadius() : Behavior
    {
        /// <summary>
        /// Scaling factor for the repulsive force applied when the agent is near a border.
        /// Higher values result in stronger repulsion forces as the agent gets closer to the border.
        /// </summary>
        private readonly float _kRepel = 10f;

        /// <summary>
        /// The desired clearance between the agent's circular hull and the border before the repulsive force is applied.
        /// If the agent gets closer than this distance to the border, the repulsion increases.
        /// </summary>
        private readonly float _safeDistance = 3f;

        /// <summary>
        /// Computes a repulsive force for the agent when near border segments.
        /// <list type="bullet">
        ///   <item><description><b>Force calculation:</b> The repulsive force is computed based on the distance between the agent's center and the closest point on each border segment.</description></item>
        ///   <item><description><b>Distance adjustment:</b> The force is adjusted by the agent's bounding radius, ensuring that the agent's proximity to the border is accounted for.</description></item>
        ///   <item><description><b>Force strength:</b> The closer the agent is to the border, the stronger the repulsion. If the agent penetrates the wall, a constant repulsive force is applied.</description></item>
        ///   <item><description><b>Inverse-square law:</b> The repulsive force follows an inverse-square law for distances within the safe threshold, becoming stronger as the agent nears the border.</description></item>
        ///   <item><description><b>Safe distance:</b> Repulsion is only applied when the agent's adjusted distance to the border is less than the defined safe distance.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx">The navigation context containing the agent and nearby borders.</param>
        internal override void Compute(NavigationContext ctx)
        {
            var totalForce = Vector2.Zero;
            var P = ctx.Agent.Center;
            var radius = ctx.Agent.Radius;

            foreach (var (A, B) in ctx.Borders)
            {
                var C = RectBody.ClosestPointOnSegment(A, B, P);
                var diff = P - C;
                var dist = diff.Length();

                if (dist <= float.Epsilon)
                    continue;

                var adjustedDistance = dist - radius;

                if (adjustedDistance < _safeDistance)
                {
                    var dir = diff / dist;

                    var mag =
                        adjustedDistance <= 0f
                        ? _kRepel * 3f
                        : _kRepel * (1f / (adjustedDistance * adjustedDistance)
                                   - 1f / (_safeDistance * _safeDistance));

                    if (mag > 0f)
                        totalForce += dir * mag;
                }
            }
            Force = totalForce;
        }
    }
}
