using Color = Simulation.Util.Color;
using Model = Simulation.Unit.Model;
using RectBody = Simulation.Unit.RectBody;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents a rectangular area in the simulation space that is off-limits for movers and other entities.
    /// 
    /// Forbidden zones are used to block areas from being traversed or occupied and are rendered visually
    /// to aid in debugging and layout planning.
    /// </summary>
    /// <param name="id">A unique identifier for the forbidden zone.</param>
    /// <param name="position">The top-left position of the zone in simulation coordinates.</param>
    /// <param name="dimension">The width and height of the forbidden zone.</param>
    internal partial class ForbiddenZone(string id, Vector2 position, Vector2 dimension) : RectBody(position, dimension)
    {
        internal string ID { get; private set; } = id;

        private Vector2 _renderDimension;

        /// <summary>
        /// Updates the rendering parameters based on the current viewport scale and offset.
        /// </summary>
        /// <param name="scale">The zoom factor used when rendering the simulation.</param>
        /// <param name="offset">The offset applied to align the simulation world to the screen.</param>
        internal void UpdateRendering(float scale)
        {
            _renderDimension = Dimension * scale;
        }

        /// <summary>
        /// Renders the forbidden zone to the screen using a dark blue rectangle.
        /// </summary>
        internal void Render()
        {
            var renderPosition = Renderer.Instance.WorldToScreen(Position);
            Renderer.Instance.DrawRectangle($"3_{ID}", renderPosition, _renderDimension, Color.BlueDark);
        }
    }
}
