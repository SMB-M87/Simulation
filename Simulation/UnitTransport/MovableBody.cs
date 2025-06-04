using Model = Simulation.Unit.Model;
using UnitRect = Simulation.Unit.UnitRect;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport
{
    /// <summary>
    /// Represents a movable rectangular body capable of path navigation and physical interaction.
    /// Inherits spatial and collision behavior from <see cref="UnitRect"/>.
    /// <list type="bullet">
    ///   <item><description><b>Acceleration:</b> The current acceleration vector applied to the body, determining its movement dynamics.</description></item>
    ///   <item><description><b>Velocity:</b> The current velocity vector of the body, influencing its direction and speed.</description></item>
    ///   <item><description><b>Destination:</b> The target position the body is moving toward.</description></item>
    ///   <item><description><b>Path:</b> A stack of intermediate waypoints used to guide the body toward its destination, aiding in pathfinding.</description></item>
    ///   <item><description><b>MaxSpeed:</b> The maximum speed at which the body can move in the simulation.</description></item>
    ///   <item><description><b>MaxForce:</b> The maximum force applied for steering, impacting the body's ability to change direction.</description></item>
    ///   <item><description><b>CellWeight:</b> Represents the weight or impact the body has on the navigable grid cells, influencing pathfinding behavior.</description></item>
    ///   <item><description><b>CollisionCountdown:</b> A timer that prevents repeated collision responses in quick succession, allowing for smoother movement.</description></item>
    ///   <item><description><b>IsBlocked:</b> Determines if the body is surrounded by obstacles (such as other agents or borders) in multiple directions, preventing movement.</description></item>
    /// </list>
    /// </summary>
    /// <param name="id">The unique identifier for this movable body. This is typically a string derived from the model type and an ID, ensuring the object is distinct within its environment.</param>
    /// <param name="model">The model type of the movable body (e.g., APM4220). It defines the general physical properties and behavior of the body.</param>
    /// <param name="position">The initial position of the body in world coordinates, representing its starting location in the simulation environment.</param>
    /// <param name="dimension">The width and height of the body, which defines its physical size and is used for collision detection and rendering.</param>
    internal class MovableBody(
        string id,
        Model model,
        Vector2 position,
        Vector2 dimension
        ) : UnitRect(
            id,
            model,
            position,
            dimension
            )
    {
        /// <summary>
        /// The current acceleration vector applied to the body.
        /// <list type="bullet">
        ///   <item><description>Represents the change in velocity applied to the body each tick, influencing how quickly the body accelerates.</description></item>
        /// </list>
        /// </summary>
        internal Vector2 Acceleration { get; set; } = Vector2.Zero;

        /// <summary>
        /// The current velocity vector of the body.
        /// <list type="bullet">
        ///   <item><description>Represents the current speed and direction of the body’s movement in the simulation.</description></item>
        /// </list>
        /// </summary>
        internal Vector2 Velocity { get; set; } = Vector2.Zero;

        /// <summary>
        /// Primary world‐space goal for this body’s movement.
        /// <list type="bullet">
        ///   <item><description>Specifies the target location in the simulation space that the body is trying to reach.</description></item>
        /// </list>
        /// </summary>
        internal Vector2 Destination { get; set; } = Vector2.Zero;

        /// <summary>
        /// Intermediate or alternative position used when swapping routes or handing off tasks.
        /// </summary>
        internal Vector2 SwapDestination { get; set; } = Vector2.Zero;

        /// <summary>
        /// Flag indicating that the <see cref="Destination"/> cannot be reached and needs to be swapped for <see cref="SwapDestination"/>.
        /// </summary>
        internal bool DestinationUnreachable { get; set; } = false;

        /// <summary>
        /// <list type="bullet">
        ///   <item><description><b>Set when a transport begins:</b> Set via <see cref="StartTransport"/> when a new transport operation is initiated.</description></item>
        ///   <item><description><b>Reset path:</b> Path is cleared during the next update cycle after the mover received an perform/execute signal.</description></item>
        /// </list>
        /// </summary>
        internal bool Reset { get; set; } = false;

        /// <summary>
        /// A stack-based path containing intermediate waypoints toward the destination.
        /// <list type="bullet">
        ///   <item><description><b>Path:</b> A sequence of waypoints that guide the body to its destination, used in pathfinding algorithms.</description></item>
        /// </list>
        /// </summary>
        internal Stack<Vector2> Path { get; set; } = [];

        /// <summary>
        /// The maximum speed this body can move.
        /// <list type="bullet">
        ///   <item><description><b>MaxSpeed:</b> Represents the fastest speed the body can travel in the simulation, influencing movement dynamics.</description></item>
        /// </list>
        /// </summary>
        internal float MaxSpeed { get; } = 5;

        /// <summary>
        /// The maximum force that can be applied to steer this body.
        /// <list type="bullet">
        ///   <item><description><b>MaxForce:</b> Controls the steering capability, determining how quickly the body can change direction.</description></item>
        /// </list>
        /// </summary>
        internal float MaxForce { get; } = 1.5f;

        /// <summary>
        /// The weight of the body on the navigable grid cells.
        /// <list type="bullet">
        ///   <item><description><b>CellWeight:</b> A value that influences the cost of movement across different cells in the grid, used in pathfinding algorithms.</description></item>
        /// </list>
        /// </summary>
        internal uint CellWeight { get; set; }

        /// <summary>
        /// A countdown timer that prevents repeated collision responses in short intervals.
        /// <list type="bullet">
        ///   <item><description><b>CollisionCountdown:</b> A timer that delays collision handling, allowing smoother movement and preventing excessive responses to collisions.</description></item>
        /// </list>
        /// </summary>
        internal uint Collided { get; set; } = 0;

        /// <summary>
        /// Determines if the agent is obstructed in at least the specified number of cardinal directions 
        /// by testing a stepped movement against static borders and neighboring agents.
        /// </summary>
        /// <param name="neighbors">List of nearby movable bodies to check for potential collisions.</param>
        /// <param name="borders">List of line segments representing static obstacles to test intersection with the agent’s bounding rectangle.</param>
        /// <param name="count">Threshold for the minimum number of blocked directions that qualifies the agent as “blocked”.</param>
        /// <param name="multiplier">Factor multiplied by the agent’s maximum speed to define the test step distance.</param>
        /// <returns>True if the number of blocked directions is greater than or equal to <paramref name="count"/>.</returns>
        internal bool IsBlocked(
            List<MovableBody> neighbors,
            List<(Vector2 A, Vector2 B)> borders,
            int count = 3,
            float multiplier = 5f
            )
        {
            var directions = new List<Vector2>
            {
                new (0, -1),
                new (1, 0),
                new (-1, 0),
                new (0, 1)
            };

            var blockedDirections = new Dictionary<Vector2, bool>();
            var step = MaxSpeed * multiplier;

            foreach (var dir in directions)
            {
                var simulatedPosition = Position + dir * step;
                var isBlocked = false;

                foreach (var (A, B) in borders)
                {
                    if (IsLineIntersectingRectangle(
                        A,
                        B,
                        simulatedPosition,
                        Dimension
                        ))
                    {
                        isBlocked = true;
                        break;
                    }
                }

                foreach (var other in neighbors)
                {
                    if (IsSATColliding(other, simulatedPosition))
                    {
                        isBlocked = true;
                        break;
                    }
                }
                blockedDirections[dir] = isBlocked;
            }

            return blockedDirections.Count(kvp => kvp.Value) >= count;
        }

        /// <summary>
        /// Evaluates movement feasibility in each of the four cardinal directions by testing a fixed offset, 
        /// returning which directions are free of collisions.
        /// </summary>
        /// <param name="neighbors">List of nearby movable bodies to check for potential collisions at the test positions.</param>
        /// <param name="borders">List of line segments representing static obstacles to test intersection with the agent’s bounding rectangle.</param>
        /// <param name="testDistance">Distance to move the agent in each direction for the collision test.</param>
        /// <returns>
        /// A dictionary mapping each cardinal direction vector (X, Y) to a boolean indicating whether that direction is free (true) or blocked (false).
        /// </returns>
        internal Dictionary<(int X, int Y), bool> GetFreeDirections(
            List<MovableBody> neighbors,
            List<(Vector2 A, Vector2 B)> borders,
            float testDistance)
        {
            var tests = new Dictionary<(int X, int Y), Vector2>
            {
                [new(0, -1)] = new(0, -1),
                [new(0, 1)] = new(0, 1),
                [new(-1, 0)] = new(-1, 0),
                [new(1, 0)] = new(1, 0),
            };

            var result = new Dictionary<(int X, int Y), bool>();

            foreach (var kv in tests)
            {
                var dir = kv.Key;
                var offset = kv.Value * testDistance;
                var newPos = Position + offset;

                var blocked = borders.Any(border =>
                    IsLineIntersectingRectangle(border.A, border.B, newPos, Dimension)
                );

                if (!blocked)
                {
                    foreach (var other in neighbors)
                    {
                        if (IsSATColliding(other, newPos))
                        {
                            blocked = true;
                            break;
                        }
                    }
                }
                result[dir] = !blocked;
            }
            return result;
        }
    }
}
