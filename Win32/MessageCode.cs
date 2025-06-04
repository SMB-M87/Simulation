namespace Win32
{
    /// <summary>
    /// Provides a collection of commonly used Windows message codes (<c>WM_*</c>) for handling system and user input events.
    /// These constants are used in the native Win32 message loop to process various events.
    /// <list type="bullet">
    ///   <item><description><b>Painting:</b> <c>WM_PAINT</c> — when part of the window needs to be redrawn.</description></item>
    ///   <item><description><b>Keyboard input:</b> <c>WM_KEYDOWN</c> — when a key is pressed.</description></item>
    ///   <item><description><b>Mouse input:</b> <c>WM_MOUSEMOVE</c>, <c>WM_LBUTTONDOWN</c>, <c>WM_LBUTTONUP</c> — mouse movement and click events.</description></item>
    ///   <item><description><b>Window management:</b> <c>WM_SIZE</c> — when the window is resized; <c>WM_DESTROY</c> — when the window is being destroyed.</description></item>
    /// </list>
    /// For a full reference, see:
    /// <see href="https://learn.microsoft.com/en-us/windows/win32/winmsg/about-messages-and-message-queues#windows-messages">Windows Messages (Win32)</see>.
    /// </summary>
    internal static class MessageCode
    {
        /// <summary>
        /// Windows message code for redrawing the window.
        /// This message is sent when a part of the window needs to be re-rendered.
        /// </summary>
        public const uint WM_PAINT = 0x000F;

        /// <summary>
        /// Windows message code for a key press on the keyboard.
        /// This message is sent when a key is pressed.
        /// </summary>
        public const uint WM_KEYDOWN = 0x0100;

        /// <summary>
        /// Windows message code for mouse movements within the window.
        /// This message is sent when the mouse cursor moves inside the window.
        /// </summary>
        public const uint WM_MOUSEMOVE = 0x0200;

        /// <summary>
        /// Windows message code for left mouse button press.
        /// This message is sent when the user clicks the left mouse button.
        /// </summary>
        public const uint WM_LBUTTONDOWN = 0x0201;

        /// <summary>
        /// Windows message code for releasing the left mouse button.
        /// This message is sent when the user releases the left mouse button.
        /// </summary>
        public const uint WM_LBUTTONUP = 0x0202;

        /// <summary>
        /// Windows message code for right mouse button press.
        /// This message is sent when the user clicks the right mouse button.
        /// </summary>
        public const uint WM_RBUTTONDOWN = 0x0204;

        /// <summary>
        /// Windows message code for releasing the right mouse button.
        /// This message is sent when the user releases the right mouse button.
        /// </summary>
        public const uint WM_RBUTTONUP = 0x0205;

        /// <summary>
        /// Windows message code for window size changes.
        /// This message is sent when the window is maximized or minimized.
        /// </summary>
        public const uint WM_SIZE = 0x0005;

        /// <summary>
        /// Windows message code for destroying a window.
        /// This message is sent when the window is closed and cleaned up.
        /// </summary>
        public const uint WM_DESTROY = 0x0002;
    }
}
