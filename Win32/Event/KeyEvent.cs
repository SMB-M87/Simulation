namespace Win32.Event
{
    /// <summary>
    /// Event data for keyboard input events.
    /// <list type="bullet">
    ///   <item><description><b>Key code:</b> Identifies the virtual key that was pressed or released.</description></item>
    ///   <item><description><b>Interaction:</b> Indicates whether the key was pressed or released.</description></item>
    ///   <item><description><b>Usage:</b> Used in event handling to respond to keyboard input in the application.</description></item>
    /// </list>
    /// </summary>
    /// <param name="keyCode">The virtual key code representing the key that was pressed or released.</param>
    /// <param name="action">The type of key interaction (<see cref="Interaction.Press"/> or <see cref="Interaction.Release"/>).</param>
    public class KeyEvent(int keyCode, Interaction action) : EventArgs
    {
        /// <summary>
        /// The virtual key code associated with the keyboard event.
        /// </summary>
        public int KeyCode { get; } = keyCode;

        /// <summary>
        /// The type of key interaction (e.g., press or release).
        /// </summary>
        public Interaction Interaction { get; } = action;
    }
}
