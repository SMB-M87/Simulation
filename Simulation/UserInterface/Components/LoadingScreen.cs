using Colors = Simulation.Util.Color;
using FontDescriptor = Simulation.Util.FontDescriptor;
using TextStyles = Simulation.Util.TextStyles;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components
{
    internal class LoadingScreen(
        string id,
        Vector2 rectRadius,
        FontDescriptor style,
        Vector4 color,
        string[] blueprints,
        bool visible = true,
        bool active = true
        ) : UIEventComponent($"{id}_loadingscreen")
    {
        private readonly Vector2 _rectRadius = rectRadius;
        private readonly string[] _blueprints = blueprints;
        private readonly bool _originalState = active;
        private readonly FontDescriptor _textStyle = style;

        private int _splashPage = 0;
        private readonly int _pageSize = 8;

        internal Vector4 Color { get; set; } = color;
        internal bool Visible { get; set; } = visible;
        internal bool Active { get; set; } = active;
        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

        internal override bool LeftClick(float X, float Y)
        {
            if (!Cycle.IsRunning && Cycle.IsRendering)
            {
                var padding = 16 * Renderer.Instance.ScaleUI;
                var textStyle = TextStyles.Readable;
                var sample = Renderer.Instance.GetTextLayout("Hg", textStyle, padding: new(0, 0));

                var lineH = sample.Y * 1.2f;
                var boxW = 400 * Renderer.Instance.ScaleUI;
                var boxH = _pageSize * lineH + padding * 2 + lineH * 1.5f;
                var vp = Renderer.Instance.GetViewport();
                var pos = new Vector2((vp.X - boxW) / 2, (vp.Y - boxH) / 2);

                var start = _splashPage * _pageSize;

                for (var i = 0; i < _pageSize; i++)
                {
                    var idx = start + i;

                    if (idx >= _blueprints.Length)
                        break;

                    var y0 = pos.Y + padding + i * lineH;

                    if (X >= pos.X + padding &&
                        X <= pos.X + padding + boxW - padding * 2 &&
                        Y >= y0 && Y <= y0 + lineH)
                    {
                        var file = _blueprints[idx];
                        Environment.Instance.LoadBlueprint(file);

                        if (Environment.Instance.Grid.Count > 0)
                        {
                            Active = false;
                            Visible = false;
                            UI.Instance.SettingButton.Visible = true;
                            UI.Instance.SettingPanel.Active = true;
                            UI.Instance.SettingButton.Render();

                            return true;
                        }
                    }
                }
                var btnY = pos.Y + padding + _pageSize * lineH + lineH * 0.5f;

                var prevX0 = pos.X + padding;
                var prevX1 = prevX0 + 60 * Renderer.Instance.ScaleUI;

                if (_splashPage > 0 &&
                    X >= prevX0 && X <= prevX1 &&
                    Y >= btnY && Y <= btnY + lineH)
                {
                    _splashPage--;
                    Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
                    Render();
                    return true;
                }

                var nextX1 = pos.X + boxW - padding;
                var nextX0 = nextX1 - 60 * Renderer.Instance.ScaleUI;
                var hasMore = (_splashPage + 1) * _pageSize < _blueprints.Length;

                if (hasMore &&
                    X >= nextX0 && X <= nextX1 &&
                    Y >= btnY && Y <= btnY + lineH)
                {
                    _splashPage++;
                    Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
                    Render();
                    return true;
                }
                return false;
            }
            return false;
        }

        internal override void Render()
        {
            if (!Cycle.IsRunning)
            {
                var padding = 16 * Renderer.Instance.ScaleUI;
                var sample = Renderer.Instance.GetTextLayout("Hg", _textStyle, new(0, 0));

                var lineH = sample.Y * 1.2f;
                var boxW = 400 * Renderer.Instance.ScaleUI;
                var boxH = _pageSize * lineH + padding * 2 + lineH * 1.5f;
                var vp = Renderer.Instance.GetViewport();
                var pos = new Vector2((vp.X - boxW) / 2, (vp.Y - boxH) / 2);

                Renderer.Instance.DrawRoundedRectangle(
                    ID,
                    pos,
                    new Vector2(boxW, boxH),
                    _rectRadius,
                    Colors.Gray85
                    );

                var start = _splashPage * _pageSize;

                for (var i = 0; i < _pageSize; i++)
                {
                    var idx = start + i;

                    if (idx >= _blueprints.Length)
                        break;

                    var text = _blueprints[idx];
                    var y = pos.Y + padding + i * lineH;

                    Renderer.Instance.DrawText(
                      id: $"{ID}_{i}",
                      text: text,
                      position: new(pos.X + padding, y),
                      padding: new(0, 0),
                      style: TextStyles.Readable,
                      color: Colors.White
                    );
                }

                var btnY = pos.Y + padding + _pageSize * lineH + lineH * 0.5f;

                Renderer.Instance.DrawText(
                  id: $"{ID}_prev",
                  text: "< Prev",
                  position: new Vector2(pos.X + padding, btnY),
                  padding: new(0, 0),
                  style: _textStyle,
                  color: _splashPage > 0 ? Colors.Cyan : Colors.Gray85
                );

                var hasMore = (_splashPage + 1) * _pageSize < _blueprints.Length;

                Renderer.Instance.DrawText(
                  id: $"{ID}_next",
                  text: "Next >",
                  position: new Vector2(pos.X + boxW - padding - 80 * Renderer.Instance.ScaleUI, btnY),
                  padding: new(0, 0),
                  style: _textStyle,
                  color: hasMore ? Colors.Cyan : Colors.Gray85
                );
            }
        }

        internal override void Reset()
        {
            Active = _originalState;
            Visible = _originalState;
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand(ID);
        }
    }
}