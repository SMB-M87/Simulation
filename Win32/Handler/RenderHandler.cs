namespace Win32.Handler
{
    /// <summary>
    /// Handler for processing the <c>WM_PAINT</c> message and triggering rendering.
    /// <list type="bullet">
    ///   <item><description><b>Scene redraw:</b> Invokes the Direct2D renderer to repaint the window contents.</description></item>
    ///   <item><description><b>Message delegation:</b> Forwards message to <c>DefWindowProc</c> after drawing.</description></item>
    ///   <item><description><b>Use case:</b> Attached to the window procedure to handle paint events.</description></item>
    /// </list>
    /// </summary>
    internal sealed class RenderHandler : IWindowHandler
    {
        /// <summary>
        /// Handles the <c>WM_PAINT</c> message by redrawing the scene.
        /// <list type="bullet">
        ///   <item><description><b>Render call:</b> Invokes <c>Direct2D.Instance.Draw()</c> to refresh all draw commands.</description></item>
        ///   <item><description><b>Default handling:</b> Calls <c>DefWindowProc</c> to complete paint processing.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The window that received the paint message.</param>
        /// <param name="msg">The message identifier (expected to be <c>WM_PAINT</c>).</param>
        /// <param name="wParam">Additional paint-related data, typically ignored.</param>
        /// <param name="lParam">Unused for <c>WM_PAINT</c>, passed through to <c>DefWindowProc</c>.</param>
        /// <returns>The result of <c>DefWindowProc</c> after handling.</returns>
        public nint HandleMessage(Window window, uint msg, nint wParam, nint lParam)
        {
            Direct2D.Instance.Render();
            return Interop.DefWindowProc(window.HWnd, msg, wParam, lParam);
        }
    }
}
