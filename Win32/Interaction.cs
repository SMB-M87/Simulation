namespace Win32
{
    /// <summary>
    /// Represents the type of user interaction with input devices, 
    /// used to distinguish between different input states for mouse and keyboard events.
    /// <list type="bullet">
    ///   <item><description><b>Press:</b> Indicates the user pressed a key or mouse button.</description></item>
    ///   <item><description><b>Release:</b> Indicates the user released a key or mouse button.</description></item>
    ///   <item><description><b>Move:</b> Indicates the user moved the mouse (no button interaction).</description></item>
    /// </list>
    /// </summary>
    public enum Interaction
    {
        None,
        Press,
        Release,
        Move
    }
}
