using System.Collections.Concurrent;
using static Vortice.Direct2D1.D2D1;
using Color4 = Vortice.Mathematics.Color4;
using DWrite = Vortice.DirectWrite.DWrite;
using Ellipse = Vortice.Direct2D1.Ellipse;
using Format = Vortice.DXGI.Format;
using HwndRenderTargetProperties = Vortice.Direct2D1.HwndRenderTargetProperties;
using ID2D1Factory = Vortice.Direct2D1.ID2D1Factory;
using ID2D1HwndRenderTarget = Vortice.Direct2D1.ID2D1HwndRenderTarget;
using IDWriteFactory = Vortice.DirectWrite.IDWriteFactory;
using IDWriteTextFormat = Vortice.DirectWrite.IDWriteTextFormat;
using Matrix3x2 = System.Numerics.Matrix3x2;
using PixelFormat = Vortice.DCommon.PixelFormat;
using PresentOptions = Vortice.Direct2D1.PresentOptions;
using Rect = Vortice.Mathematics.Rect;
using RenderTargetProperties = Vortice.Direct2D1.RenderTargetProperties;
using RoundedRectangle = Vortice.Direct2D1.RoundedRectangle;
using SizeI = Vortice.Mathematics.SizeI;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Win32
{
    /// <summary>
    /// Renderer class that uses <see href="https://github.com/amerkoleci/Vortice.Windows/tree/main/src/Vortice.Direct2D1">Vortice.Direct2D1</see>
    /// for 2D rendering in a Windows window.
    /// <list type="bullet">
    ///   <item><description><b>Graphics API:</b> Built on top of Direct2D and DirectWrite via Vortice bindings.</description></item>
    ///   <item><description><b>Capabilities:</b> Supports rendering of primitives, text, shapes, and UI elements.</description></item>
    ///   <item><description><b>Integration:</b> Uses a window-bound render target (HWND) for drawing in native Win32 apps.</description></item>
    /// </list>
    /// <para/>
    /// External references:
    /// <list type="bullet">
    ///   <item><description><b>Repository:</b> <see href="https://github.com/amerkoleci/Vortice.Windows">Vortice.Windows</see></description></item>
    ///   <item><description><b>Direct2D1 API:</b> <see href="https://github.com/amerkoleci/Vortice.Windows/blob/main/src/Vortice.Direct2D1/ID2D1RenderTarget.cs">ID2D1RenderTarget</see></description></item>
    ///   <item><description><b>DirectWrite API:</b> <see href="https://github.com/amerkoleci/Vortice.Windows/blob/main/src/Vortice.Direct2D1/DirectWrite/IDWriteFactory.cs">IDWriteFactory</see></description></item>
    /// </list>
    /// </summary>
    public sealed class Direct2D : IDisposable
    {
        private RenderTargetProperties _renderTargetProps;
        private HwndRenderTargetProperties _hwndPropsTemplate;

        /// <summary>
        /// The Direct2D render target bound to a window handle (HWND).
        /// <list type="bullet">
        ///   <item><description><b>Drawing surface:</b> All rendering operations are executed on this object.</description></item>
        ///   <item><description><b>Window-bound:</b> Tied to a specific native window handle.</description></item>
        /// </list>
        /// </summary>
        private ID2D1HwndRenderTarget _renderTarget;

        /// <summary>
        /// Factory object for creating Direct2D resources.
        /// <list type="bullet">
        ///   <item><description><b>Resource creation:</b> Generates render targets, brushes, and geometries.</description></item>
        ///   <item><description><b>Lifecycle-managed:</b> Created once during initialization.</description></item>
        /// </list>
        /// </summary>
        private readonly ID2D1Factory _d2dFactory;

        /// <summary>
        /// Factory for creating DirectWrite text formatting resources.
        /// <list type="bullet">
        ///   <item><description><b>Text support:</b> Creates text formats and layouts for rendering.</description></item>
        ///   <item><description><b>Shared factory:</b> Initialized using DirectWrite shared type.</description></item>
        /// </list>
        /// </summary>
        private readonly IDWriteFactory _dwriteFactory;

        /// <summary>
        /// Thread‐safe cache of <see cref="IDWriteTextFormat"/> instances keyed by <see cref="TextStyle"/>,
        /// to avoid repeatedly creating and disposing the same formats.
        /// </summary>
        private readonly TextFormatCache _textCache;

        /// <summary>
        /// Thread-safe collection of registered draw commands.
        /// <list type="bullet">
        ///   <item><description><b>Concurrent:</b> Supports safe access and updates from multiple threads.</description></item>
        ///   <item><description><b>Keyed actions:</b> Associates unique string keys with draw operations.</description></item>
        ///   <item><description><b>Modular:</b> Enables flexible, named rendering behavior per frame.</description></item>
        /// </list>
        /// </summary>
        private readonly ConcurrentDictionary<string, Action<ID2D1HwndRenderTarget>> _drawCommands;

        /// <summary>
        /// Synchronization object for safely enumerating or clearing draw commands.
        /// <list type="bullet">
        ///   <item><description><b>Thread-safe access:</b> Used to lock critical sections that iterate or remove commands.</description></item>
        ///   <item><description><b>Consistency:</b> Ensures rendering logic isn't disrupted by concurrent modifications.</description></item>
        /// </list>
        /// </summary>
        private readonly object _drawCommandsLock;

        /// <summary>
        /// Holds the singleton instance of the <see cref="Direct2D"/> renderer.
        /// <list type="bullet">
        ///   <item><description><b>Global access:</b> Exposed through the <see cref="Instance"/> property.</description></item>
        ///   <item><description><b>Nullable:</b> Remains <c>null</c> until <see cref="Initialize"/> is called.</description></item>
        /// </list>
        /// </summary>
        private static Direct2D? _instance;

        /// <summary>
        /// Lock object used to ensure thread-safe initialization of the singleton instance.
        /// <list type="bullet">
        ///   <item><description><b>Synchronization:</b> Prevents race conditions during renderer setup.</description></item>
        ///   <item><description><b>Private use:</b> Only used inside <see cref="Initialize"/>.</description></item>
        /// </list>
        /// </summary>
        private static readonly object _lock = new();

        /// <summary>
        /// Default background color used when clearing the screen.
        /// <list type="bullet">
        ///   <item><description><b>Consistent base:</b> Applied at the start of each draw cycle.</description></item>
        ///   <item><description><b>RGBA:</b> Fully opaque black (0, 0, 0, 1).</description></item>
        /// </list>
        /// </summary>
        private readonly Color4 _black;

        /// <summary>
        /// Gets the singleton instance of the <see cref="Direct2D"/> renderer.
        /// <list type="bullet">
        ///   <item><description><b>Singleton access:</b> Ensures a single instance of the renderer is used.</description></item>
        ///   <item><description><b>Lazy initialization:</b> Requires prior call to <see cref="Initialize"/>.</description></item>
        ///   <item><description><b>Exception:</b> Throws <see cref="InvalidOperationException"/> if accessed before initialization.</description></item>
        /// </list>
        /// </summary>
        public static Direct2D Instance
        {
            get
            {
                if (_instance == null)
                {
                    throw new InvalidOperationException("Renderer is not initialized, call the Renderer.Initialize(hWnd, rect) function.");
                }
                return _instance;
            }
        }

        /// <summary>
        /// Initializes the Direct2D rendering system.
        /// <list type="bullet">
        ///   <item><description><b>Singleton setup:</b> Instantiates the renderer if not already created.</description></item>
        ///   <item><description><b>Window binding:</b> Associates rendering with the specified window handle.</description></item>
        ///   <item><description><b>Render target size:</b> Uses <paramref name="rect"/> to define output dimensions.</description></item>
        ///   <item><description><b>Thread-safe:</b> Uses a lock to ensure only one instance is created.</description></item>
        /// </list>
        /// </summary>
        /// <param name="hWnd">Handle to the target window for rendering output.</param>
        /// <param name="rect">Client area rectangle defining the render target size.</param>
        public static void Initialize(nint hWnd, (int Left, int Right, int Top, int Bottom) rect)
        {
            lock (_lock)
            {
                _instance ??= new Direct2D(hWnd, rect);
            }
        }

        /// <summary>
        /// Creates the Direct2D and DirectWrite factories, sets up text formats, and initializes the window-bound render target.
        /// <list type="bullet">
        ///   <item><description><b>Factory initialization:</b> Creates Direct2D and DirectWrite factories for graphics and text rendering.</description></item>
        ///   <item><description><b>Render target:</b> Creates a hardware-accelerated render surface bound to the specified window handle.</description></item>
        ///   <item><description><b>Viewport sizing:</b> Calculates pixel dimensions using the provided window client rectangle.</description></item>
        /// </list>
        /// </summary>
        /// <param name="hWnd">Handle to the target window (HWND) used for binding the render target.</param>
        /// <param name="rect">Client area rectangle defining the size of the initial render target in pixels.</param>
        private Direct2D(nint hWnd, (int Left, int Right, int Top, int Bottom) rect)
        {
            _drawCommands = [];
            _drawCommandsLock = new();
            _black = new(0.0f, 0.0f, 0.0f, 1.0f);

            _d2dFactory = D2D1CreateFactory<ID2D1Factory>(Vortice.Direct2D1.FactoryType.SingleThreaded);
            _dwriteFactory = DWrite.DWriteCreateFactory<IDWriteFactory>(Vortice.DirectWrite.FactoryType.Shared);
            _textCache = new TextFormatCache(_dwriteFactory);

            _renderTargetProps = new RenderTargetProperties(new PixelFormat(Format.Unknown, Vortice.DCommon.AlphaMode.Ignore));
            _hwndPropsTemplate = new HwndRenderTargetProperties
            {
                Hwnd = hWnd,
                PresentOptions = PresentOptions.None,
                PixelSize = new SizeI(rect.Right - rect.Left, rect.Bottom - rect.Top)
            };

            _renderTarget = _d2dFactory.CreateHwndRenderTarget(_renderTargetProps, _hwndPropsTemplate);
        }

        /// <summary>
        /// Gets the current size of the render target viewport in pixels.
        /// <list type="bullet">
        ///   <item><description><b>Dimension readout:</b> Returns the width and height of the rendering surface.</description></item>
        ///   <item><description><b>Unit:</b> Measured in device pixels.</description></item>
        ///   <item><description><b>Use case:</b> Useful for layout or scaling calculations in the rendering pipeline.</description></item>
        /// </list>
        /// </summary>
        public Vector2 GetViewport() => new(_renderTarget.PixelSize.Width, _renderTarget.PixelSize.Height);

        /// <summary>
        /// Resizes the render target to match the specified width and height.
        /// <list type="bullet">
        ///   <item><description><b>Dynamic resizing:</b> Adjusts the render target to match a new window or surface size.</description></item>
        ///   <item><description><b>Reallocation:</b> May trigger reallocation of internal rendering resources.</description></item>
        ///   <item><description><b>Safe operation:</b> Intended to be called in response to resize events.</description></item>
        /// </list>
        /// </summary>
        /// <param name="width">The new width of the render target in pixels.</param>
        /// <param name="height">The new height of the render target in pixels.</param>
        public void UpdateViewport(int width, int height)
        {
            if (width <= 0 || height <= 0)
                return;

            try
            {
                _renderTarget.Resize(new SizeI(width, height));
            }
            catch (SharpGen.Runtime.SharpGenException ex)
                when ((uint)ex.HResult == 0x88990006)  // D2DERR_DISPLAY_STATE_INVALID
            {
                _renderTarget.Dispose();
                _hwndPropsTemplate.PixelSize = new SizeI(width, height);
                _renderTarget = _d2dFactory.CreateHwndRenderTarget(_renderTargetProps, _hwndPropsTemplate);
            }
        }

        /// <summary>
        /// Renders all registered draw commands.
        /// <list type="bullet">
        ///   <item><description><b>Clears:</b> Clears the screen before rendering.</description></item>
        ///   <item><description><b>Draw cycle:</b> Executes all draw actions in sorted order.</description></item>
        ///   <item><description><b>Finalizes:</b> Completes the draw operation by ending the render.</description></item>
        /// </list>
        /// </summary>
        public void Render()
        {
            _renderTarget.BeginDraw();
            _renderTarget.Clear(_black);

            KeyValuePair<string, Action<ID2D1HwndRenderTarget>>[] commands;
            lock (_drawCommandsLock)
                commands = [.. _drawCommands.OrderBy(kvp => kvp.Key)];

            foreach (var drawCommand in commands)
                drawCommand.Value?.Invoke(_renderTarget);

            _renderTarget.EndDraw();
        }

        /// <summary>
        /// Removes all draw commands from the rendering pipeline.
        /// <list type="bullet">
        ///   <item><description><b>Thread-safe:</b> Locks command list to ensure safe concurrent access.</description></item>
        ///   <item><description><b>Full clear:</b> Empties the entire <c>_drawCommands</c> collection.</description></item>
        /// </list>
        /// </summary>
        public void Clear()
        {
            lock (_drawCommandsLock)
            {
                _drawCommands.Clear();
            }
        }

        /// <summary>
        /// Registers or updates a draw command.
        /// <list type="bullet">
        ///   <item><description><b>Key-based:</b> Commands are uniquely identified by a string id.</description></item>
        ///   <item><description><b>Replaces:</b> Existing commands with the same id are overwritten.</description></item>
        ///   <item><description><b>Thread-safe:</b> Access is synchronized using a lock.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">A unique string identifier for the draw command.</param>
        /// <param name="drawAction">The action that performs drawing using the render target.</param>
        private void AddOrUpdateDrawCommand(string id, Action<ID2D1HwndRenderTarget> drawAction)
        {
            lock (_drawCommandsLock)
                _drawCommands.AddOrUpdate(id, drawAction, (_, _) => drawAction);
        }

        /// <summary>
        /// Deletes a specific draw command by its id.
        /// <list type="bullet">
        ///   <item><description><b>Targeted removal:</b> Deletes a single command associated with the given id.</description></item>
        ///   <item><description><b>Thread-safe:</b> Lock ensures safe access to the command dictionary.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">The unique id of the draw command to remove.</param>
        public void RemoveDrawCommand(string id)
        {
            lock (_drawCommandsLock)
                _drawCommands.TryRemove(id, out _);
        }

        /// <summary>
        /// Deletes all draw commands whose keys contain the specified substring.
        /// <list type="bullet">
        ///   <item><description><b>Substring match:</b> Finds all keys that include the provided string.</description></item>
        ///   <item><description><b>Batch removal:</b> Removes multiple related draw commands at once.</description></item>
        ///   <item><description><b>Thread-safe:</b> Uses a lock to avoid concurrent modification issues.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Substring used to match and remove draw command keys.</param>
        public void RemoveKeyContainedDrawCommand(string id)
        {
            lock (_drawCommandsLock)
            {
                var commands = _drawCommands.Where(kvp => kvp.Key.Contains(id)).ToList();

                foreach (var command in commands)
                    _drawCommands.TryRemove(command.Key, out _);
            }
        }

        /// <summary>
        /// Measures the rendered size of <paramref name="text"/> using a given <paramref name="style"/>,
        /// then adds optional horizontal and vertical padding.
        /// <list type="bullet">
        ///   <item>
        ///     <description><b>Text format:</b> Obtains an <see cref="IDWriteTextFormat"/> from
        ///     the internal <see cref="TextFormatCache"/> based on <paramref name="style"/>.</description>
        ///   </item>
        ///   <item>
        ///     <description><b>Layout:</b> Uses DirectWrite’s text layout to compute the raw width and height.</description>
        ///   </item>
        ///   <item>
        ///     <description><b>Padding:</b> Adds <paramref name="paddingX"/> to both left and right,
        ///     and <paramref name="paddingY"/> to both top and bottom.</description>
        ///   </item>
        /// </list>
        /// </summary>
        /// <param name="text">The string whose size you want to measure.</param>
        /// <param name="style">The <see cref="TextStyle"/> defining font family, size, weight, etc.</param>
        /// <param name="paddingX">Horizontal padding added to each side of the measured width.</param>
        /// <param name="paddingY">Vertical padding added to top and bottom of the measured height.</param>
        /// <returns>
        /// A <see cref="Vector2"/> whose X and Y components are the total width and height
        /// (including padding) required to render the text.
        /// </returns>
        public Vector2 GetTextLayout(string text, TextStyle style, Vector2 padding)
        {
            if (_dwriteFactory == null)
                return Vector2.Zero;

            var format = _textCache.Get(style);
            using var layout = _dwriteFactory.CreateTextLayout(text, format, float.MaxValue, float.MaxValue);
            var metrics = layout.Metrics;

            return new Vector2(metrics.Width + 2 * padding.X, metrics.Height + 2 * padding.Y);
        }

        /// <summary>
        /// Queues a named draw command that renders <paramref name="text"/> at the given <paramref name="position"/> 
        /// using the specified <paramref name="style"/> and <paramref name="color"/>.  
        /// Lays out the text within a <paramref name="boxW"/>×<paramref name="boxH"/> region, applies
        /// optional <paramref name="padX"/>/ <paramref name="padY"/> offsets, and—if <paramref name="centerPosition"/>—
        /// centers the text block around <paramref name="position"/> before drawing.
        /// </summary>
        /// <param name="id">Unique identifier for this draw command.</param>
        /// <param name="text">The string to render.</param>
        /// <param name="position">Top‐left (or center, if <paramref name="centerPosition"/> is true) position.</param>
        /// <param name="style">Font family, size and weight information.</param>
        /// <param name="color">Text color.</param>
        /// <param name="boxW">Maximum layout width.</param>
        /// <param name="boxH">Maximum layout height.</param>
        /// <param name="centerPosition">Whether to center the layout on <paramref name="position"/>.</param>
        /// <param name="padX">Additional horizontal offset.</param>
        /// <param name="padY">Additional vertical offset.</param>
        public void DrawText(
          string id,
          string text,
          Vector2 position,
          Vector2 padding,
          TextStyle style,
          Color4 color,
          bool centerPosition = false)
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var layout =
                _dwriteFactory.CreateTextLayout(
                  text,
                  _textCache.Get(style),
                  float.MaxValue,
                  float.MaxValue);

                var metrics = layout.Metrics;
                var pos = position + padding;

                if (centerPosition)
                {
                    pos.X -= metrics.Width * 0.5f;
                    pos.Y -= metrics.Height * 0.5f;
                }

                using var brush = rt.CreateSolidColorBrush(color);
                rt.DrawTextLayout(pos, layout, brush);
            });
        }

        /// <summary>
        /// Queues a draw command for rendering a line between two points.
        /// <list type="bullet">
        ///   <item><description><b>Keyed command:</b> Registered under <paramref name="id"/> for later updating or removal.</description></item>
        ///   <item><description><b>Line geometry:</b> Draws a straight line from <paramref name="start"/> to <paramref name="end"/>.</description></item>
        ///   <item><description><b>Style:</b> Line is rendered with the specified <paramref name="color"/> and <paramref name="thickness"/>.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Unique identifier for the draw command.</param>
        /// <param name="start">The starting point of the line.</param>
        /// <param name="end">The ending point of the line.</param>
        /// <param name="color">Color used to draw the line.</param>
        /// <param name="thickness">Thickness of the line. Default is 1.0f.</param>
        public void DrawLine(
            string id,
            Vector2 start,
            Vector2 end,
            Color4 color,
            float thickness = 1.0f)
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var brush = rt.CreateSolidColorBrush(color);
                rt.DrawLine(start, end, brush, thickness);
            });
        }

        /// <summary>
        /// Queues a draw command for rendering a rectangle at a specified position and size.
        /// <list type="bullet">
        ///   <item><description><b>Keyed command:</b> Stored using <paramref name="id"/> for updating or removal.</description></item>
        ///   <item><description><b>Dimensions:</b> Rectangle is positioned at <paramref name="position"/> with size <paramref name="dimension"/>.</description></item>
        ///   <item><description><b>Rendering style:</b> Drawn with <paramref name="color"/> and optionally filled or outlined.</description></item>
        ///   <item><description><b>Border control:</b> <paramref name="border"/> defines thickness when <paramref name="filled"/> is false.</description></item>
        ///   <item><description><b>Transform:</b> Rotated around its center by <paramref name="rotationAngle"/> degrees.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Unique identifier for the draw command.</param>
        /// <param name="position">Top-left corner of the rectangle.</param>
        /// <param name="dimension">Width and height of the rectangle.</param>
        /// <param name="color">Color used to draw the rectangle.</param>
        /// <param name="rotationAngle">Rotation angle in degrees applied around the center of the rectangle. Default is 0.</param>
        /// <param name="filled">If true, draws a filled rectangle; otherwise, draws only the border.</param>
        /// <param name="border">Thickness of the border when drawing an outlined rectangle. Default is 0.</param>
        public void DrawRectangle(
            string id,
            Vector2 position,
            Vector2 dimension,
            Color4 color,
            bool filled = true,
            float rotationAngle = 0.0f,
            float border = 0.0f)
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var brush = rt.CreateSolidColorBrush(color);

                Vector2 center = new(
                    position.X + dimension.X / 2,
                    position.Y + dimension.Y / 2
                );

                var rotationMatrix = Matrix3x2.CreateRotation(
                    rotationAngle * (MathF.PI / 180f),
                    center
                );

                rt.Transform = rotationMatrix;

                Rect rect = new(
                        position.X,
                        position.Y,
                        dimension.X,
                        dimension.Y
                        );

                if (filled)
                    rt.FillRectangle(rect, brush);
                else
                    rt.DrawRectangle(rect, brush, border);

                rt.Transform = Matrix3x2.Identity;
            });
        }

        /// <summary>
        /// Queues a draw command for rendering a rounded rectangle with specified corner radii.
        /// <list type="bullet">
        ///   <item><description><b>Keyed command:</b> Registered using <paramref name="id"/> for identification and control.</description></item>
        ///   <item><description><b>Dimensions:</b> Defined by <paramref name="position"/> and <paramref name="dimension"/>.</description></item>
        ///   <item><description><b>Corner radius:</b> Controlled by <paramref name="radius"/> (X and Y radii).</description></item>
        ///   <item><description><b>Rendering style:</b> Filled or outlined using <paramref name="filled"/> and <paramref name="border"/>.</description></item>
        ///   <item><description><b>Transform:</b> Rotated around the center using <paramref name="rotationAngle"/> in degrees.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Unique identifier for the draw command.</param>
        /// <param name="position">Top-left corner of the rounded rectangle.</param>
        /// <param name="dimension">Width and height of the rectangle.</param>
        /// <param name="radius">Corner radius in the X and Y directions.</param>
        /// <param name="color">Color used to render the shape.</param>
        /// <param name="filled">If true, fills the shape; otherwise, only the outline is drawn.</param>
        /// <param name="border">Thickness of the outline if not filled. Default is 1.0f.</param>
        /// <param name="rotationAngle">Rotation angle in degrees around the shape's center. Default is 0.</param>
        public void DrawRoundedRectangle(
            string id,
            Vector2 position,
            Vector2 dimension,
            Vector2 radius,
            Color4 color,
            bool filled = true,
            float border = 0.0f,
            float rotationAngle = 0.0f
        )
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var brush = rt.CreateSolidColorBrush(color);

                Vector2 center = new(
                    position.X + dimension.X / 2,
                    position.Y + dimension.Y / 2
                );

                var rotationMatrix = Matrix3x2.CreateRotation(
                    rotationAngle * (MathF.PI / 180f),
                    center
                );

                rt.Transform = rotationMatrix;

                RoundedRectangle roundedRect = new()
                {
                    Rect = new Rect(position.X, position.Y, dimension.X, dimension.Y),
                    RadiusX = radius.X,
                    RadiusY = radius.Y
                };

                if (filled)
                    rt.FillRoundedRectangle(roundedRect, brush);
                else
                    rt.DrawRoundedRectangle(roundedRect, brush, border);

                rt.Transform = Matrix3x2.Identity;
            });
        }

        /// <summary>
        /// Queues a draw command for rendering a circle (or ellipse with equal radii).
        /// <list type="bullet">
        ///   <item><description><b>Keyed command:</b> Identified by <paramref name="id"/> for update or removal.</description></item>
        ///   <item><description><b>Shape:</b> Draws a circle centered at <paramref name="position"/> with the given <paramref name="radius"/>.</description></item>
        ///   <item><description><b>Rendering style:</b> Filled or outlined based on <paramref name="filled"/> and <paramref name="border"/>.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Unique identifier for the draw command.</param>
        /// <param name="position">Center position of the circle.</param>
        /// <param name="radius">Radius of the circle (used for both X and Y dimensions).</param>
        /// <param name="color">Color used to render the circle.</param>
        /// <param name="filled">If true, fills the circle; otherwise, draws only the outline.</param>
        /// <param name="border">Outline thickness when <paramref name="filled"/> is false. Default is 1.0f.</param>
        public void DrawCircle(
            string id,
            Vector2 position,
            float radius,
            Color4 color,
            bool filled = true,
            float border = 0.0f
            )
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var brush = rt.CreateSolidColorBrush(color);

                Ellipse ellipse = new(
                        new Vector2(position.X, position.Y),
                        radius, radius);

                if (filled)
                    rt.FillEllipse(ellipse, brush);
                else
                    rt.DrawEllipse(ellipse, brush, border);
            });
        }

        /// <summary>
        /// Queues a draw command to render a simple slider (a line and a movable ball).
        /// <list type="bullet">
        ///   <item><description><b>Keyed command:</b> Identified by <paramref name="id"/> for updating or clearing.</description></item>
        ///   <item><description><b>Slider body:</b> Renders a horizontal line starting at <paramref name="position"/> with length <paramref name="width"/>.</description></item>
        ///   <item><description><b>Slider ball:</b> Draws a circle at <paramref name="ballX"/> to represent the slider's handle.</description></item>
        ///   <item><description><b>Custom styling:</b> Uses <paramref name="barColor"/> and <paramref name="ballColor"/> for visuals.</description></item>
        /// </list>
        /// </summary>
        /// <param name="id">Unique identifier for the draw command.</param>
        /// <param name="position">Starting point of the slider line.</param>
        /// <param name="slider">Slider metrics: {X,Y} ball radius, {Z} ball center position on x-axis, {W} width of the bar.</param>
        /// <param name="barColor">Color used to draw the slider line.</param>
        /// <param name="ballColor">Color used to fill the slider ball.</param>
        public void DrawSlider(
            string id,
            Vector2 position,
            Vector4 slider,
            Color4 barColor,
            Color4 ballColor
            )
        {
            AddOrUpdateDrawCommand(id, rt =>
            {
                using var sliderBrush = rt.CreateSolidColorBrush(barColor);
                using var ballBrush = rt.CreateSolidColorBrush(ballColor);

                rt.DrawLine(position, new Vector2(position.X + slider.W, position.Y), sliderBrush, 3.0f);

                rt.FillEllipse(new Ellipse(new Vector2(slider.Z, position.Y), slider.X, slider.Y), ballBrush);
            });
        }

        /// <summary>
        /// Disposes all unmanaged resources related to rendering.
        /// <list type="bullet">
        ///   <item><description><b>Cleanup:</b> Releases the render target, factories, and text format resources.</description></item>
        ///   <item><description><b>Memory safety:</b> Prevents memory leaks by explicitly disposing native COM objects.</description></item>
        ///   <item><description><b>Finalization:</b> Calls <see cref="GC.SuppressFinalize(object)"/> to avoid redundant finalizer execution.</description></item>
        /// </list>
        /// </summary>
        public void Dispose()
        {
            _renderTarget.Dispose();
            _d2dFactory.Dispose();
            _dwriteFactory.Dispose();
            _textCache.Dispose();

            GC.SuppressFinalize(this);
        }
    }
}
