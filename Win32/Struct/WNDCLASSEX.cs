using System.Runtime.InteropServices;

namespace Win32.Struct
{
    /// <summary>
    /// Contains information used to define a window class for the Windows API.
    /// <list type="bullet">
    ///   <item><description><b>Registration:</b> Passed to <c>RegisterClassEx</c> to register a window class.</description></item>
    ///   <item><description><b>Structure:</b> Includes pointers to icons, cursors, menu names, and the window procedure.</description></item>
    ///   <item><description><b>Layout:</b> Must set <c>cbSize</c> to <c>sizeof(WNDCLASSEX)</c> before use.</description></item>
    /// </list>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    internal struct WNDCLASSEX
    {
        /// <summary>
        /// Size of this structure in bytes.
        /// <list type="bullet">
        ///   <item><description><b>Required:</b> Must be set to <c>sizeof(WNDCLASSEX)</c> before registration.</description></item>
        ///   <item><description><b>Used by:</b> <c>RegisterClassEx</c> to validate structure layout.</description></item>
        /// </list>
        /// </summary>
        public uint cbSize;

        /// <summary>
        /// Class style flags.
        /// <list type="bullet">
        ///   <item><description><b>Behavior:</b> Determines redraw and input behavior (e.g., <c>CS_HREDRAW | CS_VREDRAW</c>).</description></item>
        ///   <item><description><b>Bitmask:</b> Combination of <c>CS_*</c> flags defined in WinUser.h.</description></item>
        /// </list>
        /// </summary>
        public uint style;

        /// <summary>
        /// Pointer to the window procedure (WndProc).
        /// <list type="bullet">
        ///   <item><description><b>Callback:</b> Function that handles window messages.</description></item>
        ///   <item><description><b>Required:</b> Must point to a valid message handler delegate.</description></item>
        /// </list>
        /// </summary>
        public nint lpfnWndProc;

        /// <summary>
        /// Extra bytes to allocate following the class structure.
        /// <list type="bullet">
        ///   <item><description><b>Usage:</b> Rarely used; typically set to 0.</description></item>
        /// </list>
        /// </summary>
        public int cbClsExtra;

        /// <summary>
        /// Extra bytes to allocate for each window instance.
        /// <list type="bullet">
        ///   <item><description><b>Usage:</b> Reserved memory for per-window data. Typically 0 unless subclassing.</description></item>
        /// </list>
        /// </summary>
        public int cbWndExtra;

        /// <summary>
        /// Handle to the application instance that registers the window class.
        /// <list type="bullet">
        ///   <item><description><b>Context:</b> Usually retrieved via <c>GetModuleHandle</c>.</description></item>
        /// </list>
        /// </summary>
        public nint hInstance;

        /// <summary>
        /// Handle to the main icon associated with the window class.
        /// <list type="bullet">
        ///   <item><description><b>Display:</b> Shown in the title bar and taskbar of top-level windows.</description></item>
        /// </list>
        /// </summary>
        public nint hIcon;

        /// <summary>
        /// Handle to the default cursor for windows of this class.
        /// <list type="bullet">
        ///   <item><description><b>Common:</b> Typically set via <c>LoadCursor</c> (e.g., <c>IDC_ARROW</c>).</description></item>
        /// </list>
        /// </summary>
        public nint hCursor;

        /// <summary>
        /// Handle to the background brush used for repainting.
        /// <list type="bullet">
        ///   <item><description><b>Effect:</b> Determines the background color or pattern.</description></item>
        ///   <item><description><b>Common:</b> Often set to <c>(HBRUSH)(COLOR_WINDOW+1)</c>.</description></item>
        /// </list>
        /// </summary>
        public nint hbrBackground;

        /// <summary>
        /// Pointer to a null-terminated string or resource name of the class menu.
        /// <list type="bullet">
        ///   <item><description><b>Optional:</b> Can be <c>IntPtr.Zero</c> for no default menu.</description></item>
        /// </list>
        /// </summary>
        public nint lpszMenuName;

        /// <summary>
        /// Pointer to the class name string.
        /// <list type="bullet">
        ///   <item><description><b>Registration:</b> Used to identify and create windows with this class.</description></item>
        /// </list>
        /// </summary>
        public nint lpszClassName;

        /// <summary>
        /// Handle to the small icon used in Alt+Tab and task switcher.
        /// <list type="bullet">
        ///   <item><description><b>Visuals:</b> Appears in the title bar and task list of small windows.</description></item>
        ///   <item><description><b>Common:</b> Often set to a smaller version of <c>hIcon</c>.</description></item>
        /// </list>
        /// </summary>
        public nint hIconSm;
    }
}
