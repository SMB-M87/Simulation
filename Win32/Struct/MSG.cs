using System.Runtime.InteropServices;

namespace Win32.Struct
{
    /// <summary>
    /// Representation of a Windows message (MSG) used in the Win32 message loop.
    /// <list type="bullet">
    ///   <item><description><b>Message data:</b> Contains window handle, message ID, parameters, timestamp, and mouse position.</description></item>
    ///   <item><description><b>Usage:</b> Retrieved with functions like <c>GetMessage</c> or <c>PeekMessage</c> and dispatched via <c>DispatchMessage</c>.</description></item>
    /// </list>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct MSG
    {
        /// <summary>
        /// Handle to the window that is the target of the message.
        /// <list type="bullet">
        ///   <item><description><b>Target:</b> Identifies which window the message is intended for.</description></item>
        /// </list>
        /// </summary>
        public nint hwnd;

        /// <summary>
        /// The message identifier code.
        /// <list type="bullet">
        ///   <item><description><b>Type:</b> Specifies the kind of message (e.g., <c>WM_PAINT</c>, <c>WM_KEYDOWN</c>).</description></item>
        /// </list>
        /// </summary>
        public uint message;

        /// <summary>
        /// Additional message-specific information.
        /// <list type="bullet">
        ///   <item><description><b>Interpretation:</b> Depends on the value of <c>message</c>.</description></item>
        ///   <item><description><b>Common usage:</b> Often used for flags or key codes.</description></item>
        /// </list>
        /// </summary>
        public nint wParam;

        /// <summary>
        /// Additional message-specific data.
        /// <list type="bullet">
        ///   <item><description><b>Interpretation:</b> Depends on the value of <c>message</c>.</description></item>
        ///   <item><description><b>Common usage:</b> Often contains coordinates or object handles.</description></item>
        /// </list>
        /// </summary>
        public nint lParam;

        /// <summary>
        /// Timestamp indicating when the message was posted.
        /// <list type="bullet">
        ///   <item><description><b>Unit:</b> Milliseconds since system startup.</description></item>
        ///   <item><description><b>Timing:</b> Useful for event ordering and input handling.</description></item>
        /// </list>
        /// </summary>
        public uint time;

        /// <summary>
        /// X-coordinate of the cursor position when the message was generated.
        /// <list type="bullet">
        ///   <item><description><b>Context:</b> Typically used with mouse-related messages.</description></item>
        /// </list>
        /// </summary>
        public int pt_x;

        /// <summary>
        /// Y-coordinate of the cursor position when the message was generated.
        /// <list type="bullet">
        ///   <item><description><b>Context:</b> Typically used with mouse-related messages.</description></item>
        /// </list>
        /// </summary>
        public int pt_y;
    }
}