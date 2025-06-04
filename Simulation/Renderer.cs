using Direct2D = Win32.Direct2D;
using FontDescriptor = Simulation.Util.FontDescriptor;
using TextStyle = Win32.TextStyle;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation
{
    internal class Renderer
    {
        private readonly Direct2D _renderer;

        private readonly Dictionary<string, TextStyle> _textStyleCache = [];
        private readonly object _textStyleCacheLock = new();

        private static Renderer? _instance;
        private static readonly object _lock = new();
        private readonly Vector2 _dimensionUI;

        internal static Renderer Instance
        {
            get
            {
                if (_instance == null)
                    throw new InvalidOperationException("Renderer is not initialized, call the Renderer.Initialize() function.");

                return _instance;
            }
        }

        internal static void Initialize(Vector2 dimension, nint HWnd, (int Left, int Right, int Top, int Bottom) rect)
        {
            lock (_lock)
                _instance ??= new Renderer(dimension, HWnd, rect);
        }

        internal Vector2 ScreenDimension { get; set; }
        internal Vector2 WorldDimension { get; set; }

        internal Vector2 Offset { get; set; }
        internal float Scale { get; set; }
        internal float ScaleUI { get; set; }

        private Renderer(Vector2 dimension, nint HWnd, (int Left, int Right, int Top, int Bottom) rect)
        {
            WorldDimension = dimension;
            _dimensionUI = dimension;
            Offset = Vector2.Zero;
            Scale = 1.0f;

            Direct2D.Initialize(HWnd, rect);
            _renderer = Direct2D.Instance;
        }

        internal Vector2 GetViewport()
        {
            return _renderer.GetViewport();
        }

        internal Vector2 WorldToScreen(Vector2 position)
        {
            return position * Scale + Offset;
        }

        internal Vector2 ScreenToWorld(Vector2 position)
        {
            return (position - Offset) / Scale;
        }

        internal void UpdateViewport(int width, int height)
        {
            if (width <= 0 || height <= 0)
                return;

            _renderer.UpdateViewport(width, height);
            ScreenDimension = GetViewport();

            Scale = MathF.Max(MathF.Min(
                width / WorldDimension.X,
                height / WorldDimension.Y),
                0.01f
                );

            Offset = new Vector2(
                (width - WorldDimension.X * Scale) / 2.0f,
                (height - WorldDimension.Y * Scale) / 2.0f
                );

            ScaleUI = Scale /
                MathF.Min(
                _dimensionUI.X / WorldDimension.X,
                _dimensionUI.Y / WorldDimension.Y
                );

            Environment.Instance.UpdateViewport(Scale);
            UI.Instance.UpdateViewport(ScaleUI);
        }

        internal void RemoveDrawCommand(string id)
        {
            _renderer.RemoveDrawCommand(id);
        }

        internal void RemoveKeyContainedDrawCommand(string id)
        {
            _renderer.RemoveKeyContainedDrawCommand(id);
        }

        internal void Clear()
        {
            _renderer.Clear();
        }

        internal void Dispose()
        {
            _renderer.Dispose();
            _textStyleCache.Clear();
        }

        internal void DrawCircle(
            string id,
           Vector2 position,
            float radius,
            Vector4 color,
            bool filled = true,
            float border = 0)
        {
            _renderer.DrawCircle(
                id,
                position,
                radius,
                new(color),
                filled,
                border);
        }

        /// <summary>
        /// Measures how big the given text will be, in device‐pixels, if rendered
        /// with <paramref name="style"/> plus the given padding on each side.
        /// </summary>
        internal Vector2 GetTextLayout(
            string text,
            FontDescriptor style,
           Vector2 padding,
            bool UI = true)
        {
            var textStyle = GetOrCreateTextStyle(style);

            if (UI)
                textStyle = textStyle.Scale(ScaleUI);
            else
                textStyle = textStyle.Scale(Scale);

            return _renderer.GetTextLayout(text, textStyle, padding);
        }

        private TextStyle GetOrCreateTextStyle(FontDescriptor fontDescriptor)
        {
            var key = $"{fontDescriptor.Family}|{fontDescriptor.SizePx}";

            lock (_textStyleCacheLock)
            {
                if (_textStyleCache.TryGetValue(key, out var existing))
                    return existing;

                var newStyle = new TextStyle(fontDescriptor.Family, fontDescriptor.SizePx);
                _textStyleCache[key] = newStyle;
                return newStyle;
            }
        }

        /// <summary>
        /// Draw command for text rendering at a given position.
        /// Defaults to top-left position unless the <paramref name="center"/> parameter is true.
        /// </summary>
        internal void DrawText(
            string id,
            string text,
           Vector2 position,
           Vector2 padding,
            FontDescriptor style,
            Vector4 color,
            bool center = false,
            bool UI = true)
        {
            var textStyle = GetOrCreateTextStyle(style);

            if (UI)
                textStyle = textStyle.Scale(ScaleUI);
            else
                textStyle = textStyle.Scale(Scale);

            _renderer.DrawText(
                id,
                text,
                position,
                padding,
                textStyle,
                new(color),
                center);
        }

        internal void DrawLine(string id, Vector2 start, Vector2 end, Vector4 color, float thickness = 1)
        {
            _renderer.DrawLine(
                id,
                start,
                end,
                new(color),
                thickness);
        }

        internal void DrawRectangle(string id, Vector2 position, Vector2 dimension, Vector4 color, bool filled = true, float rotationAngle = 0, float border = 0)
        {
            _renderer.DrawRectangle(
                id,
                position,
                dimension,
                new(color),
                filled,
                rotationAngle,
                border);
        }

        internal void DrawRoundedRectangle(string id, Vector2 position, Vector2 dimension, Vector2 radius, Vector4 color, bool filled = true, float rotationAngle = 0, float border = 0)
        {
            _renderer.DrawRoundedRectangle(
                id,
                position,
                dimension,
                radius,
                new(color),
                filled,
                rotationAngle,
                border);
        }

        /// <param name="slider">Slider metrics: {X,Y} ball radius, {W} ball center position on x-axis, Z width of the bar.</param>
        internal void DrawSlider(string id, Vector2 position, Vector4 slider, Vector4 barColor, Vector4 ballColor)
        {
            _renderer.DrawSlider(
                id,
                position,
                slider,
                new(barColor),
                new(ballColor));
        }
    }
}
