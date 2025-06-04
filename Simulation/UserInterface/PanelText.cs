using FontDescriptor = Simulation.Util.FontDescriptor;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    /// <summary>
    /// A UI component that displays a rounded‐rectangle panel containing multiple lines of text.
    /// The panel automatically sizes itself to fit its text content plus padding.
    /// </summary>
    /// <param name="id">Unique key used for grouping render commands.</param>
    /// <param name="textLines">The lines of text to display, in order.</param>
    /// <param name="style">The base text style (font, size, weight).</param>
    /// <param name="position">Top‐left position of the panel before scaling.</param>
    /// <param name="padding">Horizontal/vertical padding around the text.</param>
    /// <param name="color">Fill color of the panel.</param>
    /// <param name="textColor">Color of the text.</param>
    /// <param name="cornerRadius">Corner radius of the rounded rectangle.</param>
    internal class PanelWithText(
        string id,
        Vector2 position,
        Vector2 padding,
        Vector4 color,
        Vector4 textColor,
        FontDescriptor style,
        IReadOnlyList<string> textLines,
        bool visible = false,
        bool active = false,
        float cornerRadius = 5f
    ) : UIEventComponent($"{id}_paneltext{position.X}-{position.Y}")
    {
        private readonly Vector2 _pos = position;
        private readonly Vector2 _padding = padding;
        private readonly IReadOnlyList<string> _lines = textLines;
        private readonly float _cornerRadius = cornerRadius;
        private Vector2 _renderTextStart;

        internal Vector2 Position { get; set; } = position;
        internal Vector2 Dimension { get; set; } = padding;
        internal Vector2 Radius { get; set; } = new(cornerRadius, cornerRadius);

        internal Vector2 RenderPosition { get; set; } = position;
        internal Vector2 RenderDimension { get; set; } = padding;

        internal Vector4 Color { get; set; } = color;
        internal Vector4 TextColor { get; set; } = textColor;
        internal FontDescriptor FontDescriptor { get; set; } = style;

        internal bool Visible { get; set; } = visible;
        internal bool Active { get; set; } = active;
        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

        internal override void UpdateViewport(float scale)
        {
            RenderPosition = _pos * scale;

            var lineHeight = Renderer.Instance.GetTextLayout("Hg", FontDescriptor, new(0, 0)).Y * 1.2f;

            var widths = _lines
                .Select(l => Renderer.Instance.GetTextLayout(l, FontDescriptor, new(0, 0)).X);

            var maxWidth = widths.DefaultIfEmpty(0).Max();

            var padPx = _padding * scale;
            RenderDimension = new Vector2(maxWidth, _lines.Count * lineHeight) + padPx * 2f;

            _renderTextStart = RenderPosition + padPx;
        }

        internal override void Render()
        {
            Renderer.Instance.DrawRoundedRectangle(
                id: ID,
                position: RenderPosition,
                dimension: RenderDimension,
                radius: new Vector2(_cornerRadius, _cornerRadius),
                color: Color
            );

            var lineHeight = Renderer.Instance
                .GetTextLayout("Hg", FontDescriptor, new(0, 0)).Y * 1.2f;

            for (var i = 0; i < _lines.Count; i++)
            {
                Renderer.Instance.DrawText(
                    id: $"{ID}_line_{i}",
                    text: _lines[i],
                    position: new Vector2(
                        _renderTextStart.X,
                        _renderTextStart.Y + i * lineHeight
                    ),
                    padding: new(0, 0),
                    style: FontDescriptor,
                    color: TextColor
                );
            }
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
        }
    }
}