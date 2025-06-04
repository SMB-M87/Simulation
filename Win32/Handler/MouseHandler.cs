using MouseEvent = Win32.Event.MouseEvent;

namespace Win32.Handler
{
    /// <summary>
    /// Handler for processing mouse input within a window.
    /// <list type="bullet">
    ///   <item><description><b>Input type:</b> Handles mouse movement, button press, and release.</description></item>
    ///   <item><description><b>Interop:</b> Uses <see href="https://learn.microsoft.com/en-us/windows/win32/inputdev/virtual-key-codes">Virtual Key Codes</see> to identify messages.</description></item>
    ///   <item><description><b>Capture control:</b> Uses <c>SetCapture</c> and <c>ReleaseCapture</c> to track drag behavior.</description></item>
    /// </list>
    /// </summary>
    internal sealed class MouseHandler() : IWindowHandler
    {
        /// <summary>
        /// Processes a mouse-related window message.
        /// <list type="bullet">
        ///   <item><description><b>Coordinates:</b> Extracts mouse X and Y positions from <paramref name="lParam"/>.</description></item>
        ///   <item><description><b>Message routing:</b> Matches <paramref name="msg"/> against mouse virtual key codes.</description></item>
        ///   <item><description><b>Interaction mapping:</b> Converts message type to an <see cref="Interaction"/> value.</description></item>
        ///   <item><description><b>Event dispatch:</b> Calls <c>window.OnMouseEvent</c> with the extracted data.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The window receiving the input.</param>
        /// <param name="msg">The message identifier (e.g., mouse move or click).</param>
        /// <param name="wParam">Additional flags (e.g., mouse button state, modifiers).</param>
        /// <param name="lParam">Packed X and Y coordinates of the cursor.</param>
        /// <returns><c>IntPtr.Zero</c> if handled; otherwise, allows default processing.</returns>
        public IntPtr HandleMessage(Window window, uint msg, IntPtr wParam, IntPtr lParam)
        {
            float mouseX = unchecked((short)(lParam.ToInt32() & 0xFFFF));
            float mouseY = unchecked((short)((lParam.ToInt32() >> 16) & 0xFFFF));

            Interaction interaction;
            MouseButton button;

            switch (msg)
            {
                case VirtualKey.LEFT_BUTTON_PRESS:
                    Interop.SetCapture(window.HWnd);
                    interaction = Interaction.Press;
                    button = MouseButton.Left;
                    break;
                case VirtualKey.LEFT_BUTTON_RELEASE:
                    Interop.ReleaseCapture();
                    interaction = Interaction.Release;
                    button = MouseButton.Left;
                    break;
                case VirtualKey.RIGHT_BUTTON_PRESS:
                    Interop.SetCapture(window.HWnd);
                    interaction = Interaction.Press;
                    button = MouseButton.Right;
                    break;
                case VirtualKey.RIGHT_BUTTON_RELEASE:
                    Interop.ReleaseCapture();
                    interaction = Interaction.Release;
                    button = MouseButton.Right;
                    break;
                case VirtualKey.MOUSE_MOVE:
                    interaction = Interaction.Move;
                    button = MouseButton.None;
                    break;
                default:
                    return IntPtr.Zero;
            }
            window.OnMouseEvent(new MouseEvent(mouseX, mouseY, interaction, button));

            return nint.Zero;
        }
    }
}
