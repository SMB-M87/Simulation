using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.Unit
{
    /// <summary>
    /// Represents a basic circular entity used within the simulation, defined by a position, color and radius.
    /// 
    /// This abstract base class is intended for simulation units like producers that have a circular visual or collision shape.
    /// </summary>
    /// <param name="position">The center position of the circular body in world coordinates.</param>
    /// <param name="color">The color used for rendering the body.</param>
    /// <param name="radius">The radius of the circular body. Defaults to 0 if not specified.</param>
    internal class CircularBody(Vector2 position, Vector4 color, float radius = 0.0f)
    {
        internal Vector2 Center { get; } = position;

        internal float Radius { get; set; } = radius;

        internal Vector4 Color { get; } = color;
    }
}
