namespace Win32.Event
{
    /// <summary>
    /// Event data for window resize events.
    /// <list type="bullet">
    ///   <item><description><b>Dimensions:</b> Provides the new width and height of the window's client area.</description></item>
    ///   <item><description><b>Trigger:</b> Raised when the window receives a <c>WM_SIZE</c> message.</description></item>
    ///   <item><description><b>Usage:</b> Used to adjust rendering or layout logic after a resize.</description></item>
    /// </list>
    /// </summary>
    /// <param name="width">The new width of the window's client area.</param>
    /// <param name="height">The new height of the window's client area.</param>
    public class SizeEvent(int width, int height) : EventArgs
    {
        /// <summary>
        /// The new width of the window's client area in pixels.
        /// <list type="bullet">
        ///   <item><description><b>Measured:</b> Retrieved from the latest <c>WM_SIZE</c> message.</description></item>
        /// </list>
        /// </summary>
        public int Width { get; } = width;

        /// <summary>
        /// The new height of the window's client area in pixels.
        /// <list type="bullet">
        ///   <item><description><b>Measured:</b> Retrieved from the latest <c>WM_SIZE</c> message.</description></item>
        /// </list>
        /// </summary>
        public int Height { get; } = height;
    }
}
