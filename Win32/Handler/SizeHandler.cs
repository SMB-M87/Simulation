using SizeEvent = Win32.Event.SizeEvent;

namespace Win32.Handler
{
    /// <summary>
    /// Handles window resize events (<c>WM_SIZE</c>).
    /// <list type="bullet">
    ///   <item><description><b>Invalidation:</b> Triggers a redraw by calling <c>InvalidateRect</c>.</description></item>
    ///   <item><description><b>Viewport update:</b> Resizes the Direct2D render target to match the new client area.</description></item>
    ///   <item><description><b>Event dispatch:</b> Raises a <see cref="SizeEvent"/> to notify subscribed systems.</description></item>
    /// </list>
    /// </summary>
    internal sealed class SizeHandler : IWindowHandler
    {
        private const int _size_MINIMIZED = 1;

        /// <summary>
        /// Handles the <c>WM_SIZE</c> message to react to window resizing.
        /// <list type="bullet">
        ///   <item><description><b>Redraw:</b> Invalidates the window to trigger a repaint.</description></item>
        ///   <item><description><b>Viewport resize:</b> Updates the Direct2D render target to fit the new client size.</description></item>
        ///   <item><description><b>Event trigger:</b> Dispatches a <see cref="SizeEvent"/> to inform other components of the new dimensions.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The window that received the resize message.</param>
        /// <param name="msg">The message identifier (expected to be <c>WM_SIZE</c>).</param>
        /// <param name="wParam">Additional message-specific information (typically sizing type).</param>
        /// <param name="lParam">Contains the new width and height as packed <c>LOWORD</c> and <c>HIWORD</c>.</param>
        /// <returns>Always returns <c>IntPtr.Zero</c>, indicating the message was handled.</returns>
        public nint HandleMessage(Window window, uint msg, nint wParam, nint lParam)
        {
            var sizingType = (int)wParam;

            if (sizingType == _size_MINIMIZED)
                return IntPtr.Zero;

            Interop.GetClientRect(window.HWnd, out var rect);

            if (rect.Width == 0 || rect.Height == 0)
                return IntPtr.Zero;

            window.OnSizeEvent(new SizeEvent(rect.Width, rect.Height));

            Interop.InvalidateRect(window.HWnd, nint.Zero, false);
            return IntPtr.Zero;
        }
    }
}
