using FontDescriptor = Simulation.Util.FontDescriptor;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    /// <summary>
    /// A UI text component that displays dynamic text at a given position and style.
    /// </summary>
    /// <param name="id">Unique ID used for rendering draw command.</param>
    /// <param name="text">Initial text content of the label.</param>
    /// <param name="style">Text style definition (font, size, weight).</param>
    /// <param name="position">Base position of the label before scaling.</param>
    /// <param name="color">Text color.</param>
    internal class Text(
        string id,
        string text,
        FontDescriptor style,
        Vector2 position,
        Vector2 padding,
        Vector4 color,
        Vector4 hoverColor,
        bool visible = true,
        bool active = false
        ) : UIEventComponent($"{id}_{text}")
    {
        internal Vector2 Position { get; set; } = position;
        internal Vector2 Padding { get; set; } = padding;
        internal Vector2 RenderPosition { get; set; } = position;
        internal Vector2 RenderPadding { get; set; } = padding;
        internal Vector4 Color { get; set; } = color;
        internal Vector4 HoverColor { get; set; } = hoverColor;

        internal bool Visible { get; set; } = visible;
        internal bool Active { get; set; } = active;

        internal string Content
        {
            get => _text;
            set
            {
                _text = value;
                Render();
            }
        }
        private string _text = text;

        internal FontDescriptor Style { get; set; } = style;

        internal override void UpdateViewport(float scale)
        {
            RenderPosition = Position * scale;
        }

        internal override void Render()
        {
            Renderer.Instance.DrawText(
                $"{ID}_text",
                Content,
                RenderPosition,
                RenderPadding,
                Style,
                Color
            );
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveDrawCommand(ID);
        }
    }
}