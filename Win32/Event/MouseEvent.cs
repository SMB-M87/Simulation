namespace Win32.Event
{
    /// <summary>
    /// Event data for mouse interactions within the window.
    /// <list type="bullet">
    ///   <item><description><b>Coordinates:</b> Captures the cursor position in client coordinates.</description></item>
    ///   <item><description><b>Action type:</b> Specifies the kind of mouse interaction (e.g., press, move, release).</description></item>
    ///   <item><description><b>Usage:</b> Used in event handlers to respond to input logic.</description></item>
    /// </list>
    /// </summary>
    /// <param name="x">The horizontal mouse position relative to the window's client area.</param>
    /// <param name="y">The vertical mouse position relative to the window's client area.</param>
    /// <param name="action">The type of interaction, such as <see cref="Interaction.Press"/> or <see cref="Interaction.Move"/>.</param>
    /// <param name="button">The mouse button type, such as <see cref="MouseButton.Left"/> or <see cref="MouseButton.Right"/>.</param>
    public class MouseEvent(float x, float y, Interaction action, MouseButton button) : EventArgs
    {
        /// <summary>
        /// The X-coordinate of the mouse relative to the client area.
        /// </summary>
        public float X { get; } = x;

        /// <summary>
        /// The Y-coordinate of the mouse relative to the client area.
        /// </summary>
        public float Y { get; } = y;

        /// <summary>
        /// The type of interaction (e.g., press, move, release).
        /// </summary>
        public Interaction Interaction { get; } = action;

        /// <summary>
        /// The mouse button (e.g., left, right, none).
        /// </summary>
        public MouseButton Button { get; } = button;
    }
}
