namespace Win32.Handler
{
    /// <summary>
    /// Interface for handling Windows messages in a Win32 application.
    /// <list type="bullet">
    ///   <item><description><b>Message processing:</b> Defines a common method for responding to Win32 messages.</description></item>
    ///   <item><description><b>Custom handlers:</b> Implemented by message-specific classes (e.g. input, paint, resize).</description></item>
    ///   <item><description><b>Extensible:</b> Enables modular and testable window behavior.</description></item>
    /// </list>
    /// </summary>
    internal interface IWindowHandler
    {
        /// <summary>
        /// Handles a Windows message for a specific window.
        /// <list type="bullet">
        ///   <item><description><b>Routing:</b> Called from the main window procedure to delegate specific message handling.</description></item>
        ///   <item><description><b>Context:</b> Provides full message context via <paramref name="msg"/>, <paramref name="wParam"/>, and <paramref name="lParam"/>.</description></item>
        ///   <item><description><b>Result:</b> Return value may affect how the OS or app responds to the message.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The <see cref="Window"/> instance that received the message.</param>
        /// <param name="msg">The message identifier (e.g., <c>WM_PAINT</c>, <c>WM_KEYDOWN</c>).</param>
        /// <param name="wParam">Additional message-specific data, commonly a flag or input value.</param>
        /// <param name="lParam">Additional message-specific data, commonly packed values or pointers.</param>
        /// <returns>The result of processing the message, or <c>IntPtr.Zero</c> if handled explicitly.</returns>
        nint HandleMessage(Window window, uint msg, nint wParam, nint lParam);
    }
}
