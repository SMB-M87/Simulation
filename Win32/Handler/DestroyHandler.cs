using Win32.Event;
using GCHandle = System.Runtime.InteropServices.GCHandle;

namespace Win32.Handler
{
    /// <summary>
    /// Handler for processing the <c>WM_DESTROY</c> message when a window is being destroyed.
    /// <list type="bullet">
    ///   <item><description><b>Cleanup:</b> Frees memory stored in <c>GWLP_USERDATA</c> via <c>GCHandle</c>.</description></item>
    ///   <item><description><b>Shutdown:</b> Posts a quit message to signal application termination.</description></item>
    ///   <item><description><b>Notification:</b> Triggers <see cref="DestroyEvent"/> for cleanup logic in the application.</description></item>
    /// </list>
    /// </summary>
    /// <param name="gwl_USERDATA">The index used to access window user data (typically <c>GWLP_USERDATA</c>).</param>
    internal sealed class DestroyHandler(int gwl_USERDATA) : IWindowHandler
    {
        /// <summary>
        /// Offset used to access user-defined data in the window structure.
        /// <list type="bullet">
        ///   <item><description><b>Usage:</b> Passed to <c>GetWindowLongPtr</c> and <c>SetWindowLongPtr</c> to retrieve or clear associated <c>GCHandle</c>.</description></item>
        ///   <item><description><b>Field:</b> Usually set to <c>GWLP_USERDATA</c> for storing a pointer to managed objects.</description></item>
        ///   <item><description><b>Interop:</b> Enables linking managed instances with native window handles.</description></item>
        /// </list>
        /// </summary>
        private readonly int _gwl_USERDATA = gwl_USERDATA;

        /// <summary>
        /// Handles the <c>WM_DESTROY</c> message by cleaning up resources and posting a quit message.
        /// <list type="bullet">
        ///   <item><description><b>GCHandle cleanup:</b> Retrieves and frees any user data stored in <paramref name="window"/> using <c>GWLP_USERDATA</c>.</description></item>
        ///   <item><description><b>Nulling out:</b> Resets the window's <c>GWLP_USERDATA</c> to prevent future access.</description></item>
        ///   <item><description><b>Event trigger:</b> Raises <see cref="DestroyEvent"/> to allow the app to react to destruction.</description></item>
        ///   <item><description><b>Exit signal:</b> Calls <c>PostQuitMessage</c> to start shutdown sequence.</description></item>
        /// </list>
        /// </summary>
        /// <param name="window">The window receiving the <c>WM_DESTROY</c> message.</param>
        /// <param name="msg">The message ID (<c>WM_DESTROY</c>).</param>
        /// <param name="wParam">Unused for <c>WM_DESTROY</c>.</param>
        /// <param name="lParam">Unused for <c>WM_DESTROY</c>.</param>
        /// <returns>Always returns <c>IntPtr.Zero</c>.</returns>
        public nint HandleMessage(Window window, uint msg, nint wParam, nint lParam)
        {
            var ptr = Interop.GetWindowLongPtr(window.HWnd, _gwl_USERDATA);
            if (ptr != nint.Zero)
            {
                var handle = GCHandle.FromIntPtr(ptr);

                if (handle.IsAllocated)
                    handle.Free();
            }
            Interop.SetWindowLongPtr(window.HWnd, _gwl_USERDATA, nint.Zero);

            window.OnDestroyEvent(new DestroyEvent());

            Interop.PostQuitMessage(0);

            return nint.Zero;
        }
    }
}
