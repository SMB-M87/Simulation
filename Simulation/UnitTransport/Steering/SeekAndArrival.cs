using ActorRefs = Akka.Actor.ActorRefs;
using NavigableGrid = Simulation.Scene.NavigableGrid;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.Steering
{
    /// <summary>
    /// Steering behavior that directs an agent toward its target destination, dynamically advances path waypoints.
    /// The agent slows down smoothly when approaching the destination (arrival behavior).
    /// Includes basic collision courtesy to avoid reaching a target at the same time as another agent.
    /// 
    /// <list type="bullet">
    ///   <item><description><b>Arrival Behavior:</b> Smooth deceleration as the agent nears the target.</description></item>
    ///   <item><description><b>Path Navigation:</b> Continuously updates the agent's path, popping waypoints as it gets closer.</description></item>
    ///   <item><description><b>Collision Avoidance:</b> Prevents multiple agents from arriving at the same location simultaneously.</description></item>
    ///   <item><description><b>Proximity Threshold:</b> A defined distance at which the agent slows down as it approaches the destination.</description></item>
    ///   <item><description><b>Force Calculation:</b> Computes the force required to move the agent towards the destination, considering current velocity and max force.</description></item>
    /// </list>
    /// </summary>
    internal class SeekAndArrival() : Behavior
    {
        /// <summary>
        /// Factor that influences the proximity threshold used in arrival behavior.
        /// Determines how close the agent must be to the target before slowing down.
        /// <list type="bullet">
        ///   <item><description><b>Proximity Factor:</b> A multiplier applied to the agent’s dimension to calculate the proximity threshold.</description></item>
        ///   <item><description><b>Effect on Arrival:</b> A lower value allows the agent to slow down earlier, while a higher value means the agent only decelerates closer to the destination.</description></item>
        /// </list>
        /// </summary>
        private readonly float _proximityFactor = 1.0f;

        /// <summary>
        /// Determines and applies the appropriate steering force: selects the next waypoint or final target, 
        /// removes completed waypoints, smoothly slows the agent when close, respects maximum speed and force limits, 
        /// and aborts if the path is currently blocked by another agent.
        /// </summary>
        /// <param name="ctx">The navigation context, containing information about the agent's state, its destination, and neighboring agents.</param>
        internal override void Compute(NavigationContext ctx)
        {
            if (ctx.Agent.Destination == Vector2.Zero)
            {
                Force = Vector2.Zero;
                return;
            }

            var arrival = true;
            var dest = ctx.Agent.Destination;

            if (ctx.Agent.Path.Count > 0)
            {
                dest = ctx.Agent.Path.Peek();
                var dist = (dest - ctx.Agent.Center).Length();

                if (ctx.Agent.Path.Count > 1 &&
                       dist < ctx.Agent.Radius + ctx.Agent.MaxSpeed)
                {
                    ctx.Agent.Path.Pop();
                    dest = ctx.Agent.Path.Peek();
                }
                arrival = ctx.Agent.Path.Count <= 1;
            }

            if (DestinationBlocked(ctx.Agent, ctx.Neighbors, ctx.Environment))
                return;

            var desired = dest - ctx.Agent.Center;
            var distance = desired.Length();

            if (distance > 0)
            {
                desired /= distance;

                var length = ctx.Agent.Dimension.Length() * 0.45f;

                if (arrival && distance < length)
                    desired *= ctx.Agent.MaxSpeed * (distance / length);
                else
                    desired *= ctx.Agent.MaxSpeed;

                var force = desired - ctx.Agent.Velocity;

                if (force.Length() > ctx.Agent.MaxForce)
                    force = Vector2.Normalize(force) * ctx.Agent.MaxForce;

                Force = force;
            }
            else
                Force = Vector2.Zero;
        }

        /// <summary>
        /// Checks if the agent’s original or alternate target is occupied by nearby agents; 
        /// if so, pauses movement, optionally swaps or recalculates the destination to a less congested position, 
        /// and signals when the route is clear.
        /// </summary>
        private bool DestinationBlocked(
            MovableBody agent,
            List<MovableBody> neighbors,
            NavigableGrid environment
            )
        {
            if (agent.ServiceRequester == ActorRefs.Nobody)
                return false;

            var swapOGDestination = false;
            var swapTemp = false;
            var OGDestination = agent.DestinationUnreachable ? agent.SwapDestination : agent.Destination;
            var tempDestination = agent.DestinationUnreachable ? agent.Destination : agent.SwapDestination;

            if (tempDestination == Vector2.Zero)
                tempDestination = OGDestination;

            foreach (var other in neighbors)
            {
                if (OGDestination != Vector2.Zero)
                    swapOGDestination =
                        CheckDestination(OGDestination, agent.Center, agent.Dimension,
                                         other.Center, other.Dimension);

                if (tempDestination != Vector2.Zero && OGDestination != tempDestination)
                    swapTemp = CheckDestination(tempDestination, agent.Center, agent.Dimension,
                                                other.Center, other.Dimension);

                if (swapOGDestination || swapTemp)
                    break;
            }

            if (swapOGDestination)
            {
                if (!agent.DestinationUnreachable)
                {
                    Force = Vector2.Zero;
                    agent.Velocity = new Vector2(0.00000001f, 0.00000001f);
                    agent.DestinationUnreachable = true;
                    agent.Reset = true;

                    if (agent.SwapDestination == Vector2.Zero)
                    {
                        agent.SwapDestination = agent.Destination;
                        agent.Destination = environment.GetLeastCrowdedNearbyPosition(
                            agent.SwapDestination,
                            agent.Position,
                            agent.Dimension,
                            agent.CellWeight
                            );
                    }
                    return true;
                }
            }
            else if (agent.DestinationUnreachable)
            {
                agent.Destination = agent.SwapDestination;
                agent.SwapDestination = Vector2.Zero;
                agent.DestinationUnreachable = false;
                agent.Reset = true;
                return true;
            }

            if (swapTemp)
            {
                Force = Vector2.Zero;
                agent.Velocity = new Vector2(0.00000001f, 0.00000001f);
                agent.Reset = true;
                agent.Destination = environment.GetLeastCrowdedNearbyPosition(
                    agent.SwapDestination,
                    agent.Position,
                    agent.Dimension,
                    agent.CellWeight
                    );

                return true;
            }
            return false;
        }

        /// <summary>
        /// Evaluates whether two agents are both en route to conflicting destinations within each 
        /// other’s proximity thresholds, indicating a potential arrival collision.
        /// </summary>
        private bool CheckDestination(
            Vector2 destination,
            Vector2 center,
            Vector2 dimension,
            Vector2 otherCenter,
            Vector2 otherDimension
            )
        {
            var otherProximityToAgentDestination = (destination - otherCenter).Length();
            var otherProximity = otherDimension.Length() * _proximityFactor;

            var proximityToDestination = (destination - center).Length();
            var proximity = dimension.Length() * _proximityFactor;

            if ((otherProximityToAgentDestination <= otherProximity &&
                otherProximityToAgentDestination - otherProximity <= proximityToDestination - proximity))
            {
                return true;
            }
            return false;
        }
    }
}
