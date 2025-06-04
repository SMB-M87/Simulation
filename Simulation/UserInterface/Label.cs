using FontDescriptor = Simulation.Util.FontDescriptor;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    internal class Label(
        string id,
        string text,
        FontDescriptor style,
        Vector2 position,
        Vector2 padding,
        Vector4 textColor,
        Vector4 backgroundColor,
        bool visible = true
        ) : UIEventComponent($"{id}_button")
    {
        private readonly bool _originalVisibleState = visible;
        private readonly FontDescriptor _originalTextSyle = style;

        internal Vector2 Position { get; set; } = position;
        internal Vector2 Dimension { get; set; } = padding;
        internal Vector2 RenderPosition { get; set; } = position;
        internal Vector2 RenderDimension { get; set; } = padding;
        private Vector2 RenderTextPosition { get; set; } = position;
        internal Vector2 RenderCorner { get; set; }
        internal Vector4 BackgroundColor { get; set; } = backgroundColor;

        internal string Content
        {
            get
            {
                return _context;
            }
            set
            {
                _context = value;

                UpdateViewport(Renderer.Instance.ScaleUI);
                Render();
            }
        }
        private string _context = text;

        internal FontDescriptor TextStyle { get; set; } = style;
        internal Vector4 TextColor { get; set; } = textColor;

        internal bool Visible { get; set; } = visible;
        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

        internal override void UpdateViewport(float scale)
        {
            RenderPosition = Position * scale;

            RenderDimension = Renderer.Instance.GetTextLayout(
                Content, TextStyle, new(Dimension.X * scale, Dimension.Y * scale)
            );

            var rawSize = Renderer.Instance.GetTextLayout(Content, TextStyle, new(0, 0));

            var extra = (RenderDimension - rawSize) * 0.5f;
            var corner = TextStyle.SizePx * 0.25f;
            RenderCorner = new Vector2(corner, corner);
            RenderTextPosition = RenderPosition + extra;
        }

        internal override void Render()
        {
            Renderer.Instance.DrawRoundedRectangle(
                id: $"{ID}_area",
                position: RenderPosition,
                dimension: RenderDimension,
                radius: RenderCorner,
                color: BackgroundColor
            );

            Renderer.Instance.DrawText(
                id: $"{ID}_text",
                text: Content,
                position: RenderTextPosition,
                padding: new(0, 0),
                style: TextStyle,
                color: TextColor
            );
        }

        internal override void Reset()
        {
            if (Visible != _originalVisibleState)
                Visible = _originalVisibleState;

            TextStyle = _originalTextSyle;
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
        }
    }
}