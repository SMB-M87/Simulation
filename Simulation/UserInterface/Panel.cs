using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    /// <summary>
    /// A UI panel component that displays a rounded rectangle background at a given position and size.
    /// Typically used as a visual container for other UI elements.
    /// </summary>
    /// <param name="id">Unique ID used for rendering draw command.</param>
    /// <param name="position">Base position of the panel before scaling.</param>
    /// <param name="dimension">Base size of the panel before scaling.</param>
    /// <param name="radius">Corner radius of the panel before scaling.</param>
    /// <param name="color">Background color of the panel.</param>
    internal class Panel(
        string id,
        Vector2 position,
        Vector2 dimension,
        Vector2 radius,
        Vector4 color,
        Vector4 hoverColor,
        bool visible = false,
        bool active = false
    ) : UIEventComponent($"{id}_panel")
    {
        private readonly bool _originalVisible = visible;
        private readonly bool _originalActive = active;

        internal Vector2 Position { get; set; } = position;
        internal Vector2 Dimension { get; set; } = dimension;
        private readonly Vector2 _radius = radius;

        internal Vector2 RenderPosition { get; set; } = position;
        internal Vector2 RenderDimension { get; set; } = dimension;
        internal Vector2 RenderRadius { get; set; } = radius;

        internal Vector4 Color { get; set; } = color;
        internal Vector4 HoverColor { get; set; } = hoverColor;

        internal bool Visible { get; set; } = visible;
        internal bool Active { get; set; } = active;
        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

        internal override bool LeftClick(float X, float Y)
        {
            if (X < RenderPosition.X || X > RenderPosition.X + RenderDimension.X ||
                Y < RenderPosition.Y || Y > RenderPosition.Y + RenderDimension.Y)
                return false;

            OnLeftClick();
            return true;
        }

        internal override void OnLeftClick()
        {
            Clicked = true;
        }

        internal override bool LeftRelease()
        {
            if (Clicked)
            {
                OnLeftRelease();
                return true;
            }
            return false;
        }

        internal override void OnLeftRelease()
        {
            Clicked = false;
        }

        internal override void Hover(float X, float Y)
        {
            if (X < RenderPosition.X || X > RenderPosition.X + RenderDimension.X ||
                Y < RenderPosition.Y || Y > RenderPosition.Y + RenderDimension.Y)
            {
                OnHover();
                return;
            }
        }

        internal override void OnHover()
        {
            Hovered = !Hovered;
        }

        internal override void UpdateViewport(float scale)
        {
            RenderPosition = Position * scale;
            RenderDimension = Dimension * scale;
            RenderRadius = _radius * scale;
        }

        internal override void Render()
        {
            Renderer.Instance.DrawRoundedRectangle(
                ID,
                RenderPosition,
                RenderDimension,
                RenderRadius,
                Color
            );
        }

        internal override void Reset()
        {
            base.Reset();

            Visible = _originalVisible;
            Active = _originalActive;
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
        }
    }
}