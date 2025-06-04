using FontDescriptor = Simulation.Util.FontDescriptor;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    internal class Button(
        string id,
        string text,
        FontDescriptor style,
        Vector2 position,
        Vector2 padding,
        Vector4 textColor,
        Vector4 textHoverColor,
        Vector4 buttonColor,
        Vector4 hoverColor,
        bool visible = true,
        bool active = false
        ) : UIEventComponent($"{id}_button")
    {
        private readonly bool _originalToggleState = active;
        private readonly bool _originalVisibleState = visible;
        private readonly FontDescriptor _originalTextSyle = style;

        internal Vector2 Position { get; set; } = position;
        internal Vector2 Dimension { get; set; } = padding;
        internal Vector2 RenderPosition { get; set; } = position;
        internal Vector2 RenderDimension { get; set; } = padding;
        private Vector2 RenderTextPosition { get; set; } = position;
        internal Vector2 RenderCorner { get; set; }
        internal Vector4 Color { get; set; } = buttonColor;
        internal Vector4 HoverColor { get; set; } = hoverColor;

        internal string Text { get; set; } = text;
        internal FontDescriptor TextStyle { get; set; } = style;
        internal Vector4 TextColor { get; set; } = textColor;
        internal Vector4 TextHoverColor { get; set; } = textHoverColor;

        internal bool Visible { get; set; } = visible;
        internal bool Active { get; set; } = active;
        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

        internal override bool LeftClick(float X, float Y)
        {
            if (X >= RenderPosition.X && X <= RenderPosition.X + RenderDimension.X &&
                Y >= RenderPosition.Y && Y <= RenderPosition.Y + RenderDimension.Y)
            {
                OnLeftClick();
                return true;
            }
            return false;
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

        internal override void Hover(float mouseX, float mouseY)
        {
            var wasHovering = Hovered;

            Hovered =
                mouseX >= RenderPosition.X &&
                mouseX <= RenderPosition.X + RenderDimension.X &&
                mouseY >= RenderPosition.Y &&
                mouseY <= RenderPosition.Y + RenderDimension.Y;

            if (Hovered)
            {
                OnHover();
            }
            else if (Hovered != wasHovering)
                Render();
        }

        internal override void OnHover()
        {
            Render();
        }

        internal override void UpdateViewport(float scale)
        {
            RenderPosition = Position * scale;

            RenderDimension = Renderer.Instance.GetTextLayout(
                Text, TextStyle, new(Dimension.X * scale, Dimension.Y * scale)
            );

            var rawSize = Renderer.Instance.GetTextLayout(Text, TextStyle, new(0, 0));

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
                color: Hovered ? HoverColor : Color
            );

            Renderer.Instance.DrawText(
                id: $"{ID}_text",
                text: Text,
                position: RenderTextPosition,
                padding: new(0, 0),
                style: TextStyle,
                color: Hovered ? TextHoverColor : TextColor
            );
        }

        internal override void Reset()
        {
            if (Active != _originalToggleState)
                Active = _originalToggleState;

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