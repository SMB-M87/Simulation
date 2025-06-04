namespace Win32.Event
{
    /// <summary>
    /// Event data signaling that a window is being destroyed.
    /// <list type="bullet">
    ///   <item><description><b>Trigger:</b> Raised in response to the <c>WM_DESTROY</c> message.</description></item>
    ///   <item><description><b>Purpose:</b> Used to perform cleanup tasks before the application shuts down.</description></item>
    ///   <item><description><b>Payload:</b> Contains no additional data; serves only as a notification.</description></item>
    /// </list>
    /// </summary>
    public class DestroyEvent() : EventArgs { }
}
