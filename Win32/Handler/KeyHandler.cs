using KeyEvent = Win32.Event.KeyEvent;

namespace Win32.Handler
{
    /// <summary>
    /// Handler for processing keyboard input within a window.
    /// <list type="bullet">
    ///   <item><description><b>Input source:</b> Responds to key press and release messages.</description></item>
    ///   <item><description><b>Interop:</b> Uses <see href="https://learn.microsoft.com/en-us/windows/win32/inputdev/virtual-key-codes">Virtual Key Codes</see> for key identification.</description></item>
    ///   <item><description><b>Dispatch:</b> Sends <see cref="KeyEvent"/> to the window for application-level handling.</description></item>
    /// </list>
    /// </summary>
    internal sealed class KeyHandler : IWindowHandler
    {
        /// <summary>
        /// Processes keyboard messages and converts them into a <see cref="KeyEvent"/>.
        /// <list type="bullet">
        ///   <item><description><b>Message type:</b> Supports both key press and key release via <paramref name="msg"/>.</description></item>
        ///   <item><description><b>Key code:</b> Extracted from <paramref name="wParam"/> and passed to the event.</description></item>
        ///   <item><description><b>Interaction:</b> Interpreted as <see cref="Interaction.Press"/> or <see cref="Interaction.Release"/>.</description></item>
        ///   <item><description><b>Event dispatch:</b> Triggers <c>window.OnKeyEvent</c> with the constructed key event.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The window that received the keyboard message.</param>
        /// <param name="msg">The message identifier (<c>WM_KEYDOWN</c>, <c>WM_KEYUP</c>, etc.).</param>
        /// <param name="wParam">Specifies the virtual-key code of the key.</param>
        /// <param name="lParam">Additional key data (not used in this implementation).</param>
        /// <returns>Always returns <c>IntPtr.Zero</c> after handling.</returns>
        public nint HandleMessage(Window window, uint msg, nint wParam, nint lParam)
        {
            var keyCode = wParam.ToInt32();

            var interaction = msg switch
            {
                VirtualKey.KEY_PRESS => Interaction.Press,
                VirtualKey.KEY_RELEASE => Interaction.Release,
                _ => Interaction.None,
            };

            var keyEvent = new KeyEvent(keyCode, interaction);
            window.OnKeyEvent(keyEvent);

            return nint.Zero;
        }
    }
}
