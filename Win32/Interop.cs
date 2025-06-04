using CallingConvention = System.Runtime.InteropServices.CallingConvention;
using LibraryImportAttribute = System.Runtime.InteropServices.LibraryImportAttribute;
using MarshalAsAttribute = System.Runtime.InteropServices.MarshalAsAttribute;
using MSG = Win32.Struct.MSG;
using RECT = Win32.Struct.RECT;
using StringMarshalling = System.Runtime.InteropServices.StringMarshalling;
using UnmanagedFunctionPointerAttribute = System.Runtime.InteropServices.UnmanagedFunctionPointerAttribute;
using UnmanagedType = System.Runtime.InteropServices.UnmanagedType;

namespace Win32
{
    /// <summary>
    /// Contains P/Invoke definitions for various native Windows API functions, enabling direct interop access for:
    /// <list type="bullet">
    ///   <item><description><b>Window management:</b> Create, register, and modify Win32 windows (e.g., <c>RegisterClassEx</c>, <c>CreateWindowEx</c>).</description></item>
    ///   <item><description><b>Input processing:</b> Handle mouse and keyboard input, system metrics, and cursor control.</description></item>
    ///   <item><description><b>Message loop:</b> Fetch and dispatch messages for the Windows message queue (<c>GetMessage</c>, <c>DispatchMessage</c>).</description></item>
    ///   <item><description><b>Rendering:</b> Trigger painting and retrieve client rectangles for drawing with Direct2D.</description></item>
    ///   <item><description><b>Interop safety:</b> Uses <see cref="LibraryImportAttribute"/> for AOT-friendly and source-generated P/Invoke (preferred over <c>DllImport</c> in .NET 7+).</description></item>
    /// </list>
    /// This static class centralizes all unmanaged Win32 function calls and avoids the need for instantiating interop wrappers.
    /// <para>
    /// For more information, refer to:
    /// <see href="https://learn.microsoft.com/en-us/dotnet/standard/native-interop/pinvoke">P/Invoke (Microsoft Docs)</see>
    /// and
    /// <see href="https://learn.microsoft.com/en-us/windows/win32/apiindex/windows-api-list">Windows API Reference</see>.
    /// </para>
    /// </summary>
    public static partial class Interop
    {
        /// <summary>
        /// The name of the User32 DLL, which contains functions for window management and user interface functionalities.
        /// </summary>
        private const string _user32 = "user32.dll";

        /// <summary>
        /// The name of the Kernel32 DLL, which provides core functions for process management, memory management and operating system functionality.
        /// </summary>
        private const string _kernel32 = "kernel32.dll";

        /// <summary>
        /// Indicates that the image being loaded is an icon (ICON).
        /// Used as a parameter for the LoadImage function.
        /// </summary>
        internal const uint _image_ICON = 1;

        /// <summary>
        /// Indicates that the image should be loaded from a file rather than from a resource.
        /// Used as a flag in the LoadImage function.
        /// </summary>
        internal const uint _lr_LOADFROMFILE = 0x00000010;

        /// <summary>
        /// Retrieves a handle to a loaded module (DLL or EXE) in the current process.
        /// </summary>
        /// <param name="lpModuleName">
        /// The name of the module (e.g., "user32.dll" or "kernel32.dll"). 
        /// Use null to retrieve a handle to the executable module of the current process.
        /// </param>
        /// <returns>
        /// A handle to the module if found, or IntPtr.Zero in case of an error. 
        /// Call GetLastError to obtain the error code.
        /// </returns>
        [LibraryImport(_kernel32, EntryPoint = "GetModuleHandleW", SetLastError = true, StringMarshalling = StringMarshalling.Utf16)]
        internal static partial IntPtr GetModuleHandle(string? lpModuleName);

        /// <summary>
        /// Delegate representing a window procedure (WndProc) for processing Windows messages.
        /// Used in interop scenarios where a callback is required for native Windows API calls.
        /// </summary>
        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        internal delegate IntPtr WndProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);

        /// <summary>
        /// Registers an extended window class (WNDCLASSEX) for use with CreateWindowEx.
        /// This enables the application to create a window with custom properties.
        /// </summary>
        /// <param name="lpWndClass">A pointer to a WNDCLASSEX structure containing the window class properties.</param>
        /// <returns>A class ID (ATOM) if registration succeeds, otherwise 0 on failure.</returns>
        [LibraryImport(_user32, EntryPoint = "RegisterClassExW", SetLastError = true)]
        internal static partial ushort RegisterClassEx(IntPtr lpWndClass);

        /// <summary>
        /// Creates a new window with the specified class, title, style and position.
        /// This is an advanced version of CreateWindow, allowing additional window properties to be set.
        /// </summary>
        /// <param name="dwExStyle">Extended window style options (e.g., WS_EX_TOPMOST).</param>
        /// <param name="lpClassName">The name of the previously registered window class.</param>
        /// <param name="lpWindowName">The title of the window.</param>
        /// <param name="dwStyle">The window style (e.g., WS_OVERLAPPEDWINDOW).</param>
        /// <param name="x">The x-position of the window on the screen.</param>
        /// <param name="y">The y-position of the window on the screen.</param>
        /// <param name="nWidth">The width of the window in pixels.</param>
        /// <param name="nHeight">The height of the window in pixels.</param>
        /// <param name="hWndParent">The handle to the parent window, or IntPtr.Zero for a top-level window.</param>
        /// <param name="hMenu">The handle to the window's menu, or IntPtr.Zero if there is no menu.</param>
        /// <param name="hInstance">The handle to the application instance.</param>
        /// <param name="lpParam">An optional parameter passed to the window procedure, or IntPtr.Zero.</param>
        /// <returns>A handle (hWnd) to the created window, or IntPtr.Zero on failure.</returns>
        [LibraryImport(_user32, EntryPoint = "CreateWindowExW", StringMarshalling = StringMarshalling.Utf16, SetLastError = true)]
        internal static partial IntPtr CreateWindowEx(
            int dwExStyle,
            [MarshalAs(UnmanagedType.LPWStr)] string lpClassName,
            [MarshalAs(UnmanagedType.LPWStr)] string lpWindowName,
            uint dwStyle,
            int x, int y,
            int nWidth, int nHeight,
            IntPtr hWndParent,
            IntPtr hMenu,
            IntPtr hInstance,
            IntPtr lpParam
            );

        /// <summary>
        /// The default window procedure that processes messages not explicitly handled by the application.
        /// This ensures that unprocessed messages are correctly passed to the operating system.
        /// </summary>
        /// <param name="hWnd">The handle to the window receiving the message.</param>
        /// <param name="msg">The type of message being processed.</param>
        /// <param name="wParam">Additional information about the message, depending on its type.</param>
        /// <param name="lParam">Additional message data, depending on its type.</param>
        /// <returns>The processing results of the message.</returns>
        [LibraryImport(_user32, EntryPoint = "DefWindowProcW", SetLastError = true)]
        internal static partial IntPtr DefWindowProc(
            IntPtr hWnd,
            uint msg,
            IntPtr wParam,
            IntPtr lParam
            );

        /// <summary>
        /// Retrieves a property or setting of a window, such as the style or a pointer to user data.
        /// </summary>
        /// <param name="hWnd">The handle to the window.</param>
        /// <param name="nIndex">The offset of the property to retrieve (e.g., GWL_STYLE, GWL_EXSTYLE).</param>
        /// <returns>The value of the retrieved property, or IntPtr.Zero on failure.</returns>
        [LibraryImport(_user32, EntryPoint = "GetWindowLongPtrW", SetLastError = true)]
        internal static partial IntPtr GetWindowLongPtr(IntPtr hWnd, int nIndex);

        /// <summary>
        /// Modifies a property or setting of a window, such as the style or a pointer to user data.
        /// </summary>
        /// <param name="hWnd">The handle to the window.</param>
        /// <param name="nIndex">The offset of the property to modify (e.g., GWL_STYLE, GWL_EXSTYLE).</param>
        /// <param name="dwNewLong">The new value to set.</param>
        /// <returns>The previous value of the modified property, or IntPtr.Zero on failure.</returns>
        [LibraryImport(_user32, EntryPoint = "SetWindowLongPtrW", SetLastError = true)]
        internal static partial IntPtr SetWindowLongPtr(
            IntPtr hWnd,
            int nIndex,
            IntPtr dwNewLong
            );

        /// <summary>
        /// Loads a cursor from the application's resources or a system-defined standard cursor.
        /// </summary>
        /// <param name="hInstance">The handle to the executable instance containing the cursor resource, or IntPtr.Zero for a standard cursor.</param>
        /// <param name="lpCursorName">The ID or name of the cursor. Use a predefined value (e.g., IDC_ARROW) for standard cursors.</param>
        /// <returns>A handle to the loaded cursor, or IntPtr.Zero on failure.</returns>
        [LibraryImport(_user32, EntryPoint = "LoadCursorW", SetLastError = true)]
        internal static partial IntPtr LoadCursor(IntPtr hInstance, int lpCursorName);

        /// <summary>
        /// Shows or hides the mouse cursor depending on the specified parameter.
        /// </summary>
        /// <param name="bShow">True to show the cursor, false to hide it.</param>
        /// <returns>The new visibility state of the cursor as a boolean value.</returns>
        [LibraryImport(_user32, EntryPoint = "LoadCursorW", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool ShowCursor([MarshalAs(UnmanagedType.Bool)] bool bShow);

        /// <summary>
        /// Forces mouse input to be directed to the specified window, even if the mouse is outside the window.
        /// This is often used for implementing custom window dragging or drag-and-drop functionality.
        /// </summary>
        /// <param name="hWnd">The handle of the window that should receive the mouse input.</param>
        /// <returns>A handle to the window that previously received the input.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        internal static partial IntPtr SetCapture(IntPtr hWnd);

        /// <summary>
        /// Restores the default mouse input handling by releasing the mouse capture.
        /// This allows input to be captured by the window under the mouse.
        /// </summary>
        /// <returns>True if the input was successfully released, otherwise false.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool ReleaseCapture();

        /// <summary>
        /// Changes the visibility and status of a window, such as maximizing, minimizing, or hiding.
        /// </summary>
        /// <param name="hWnd">The handle to the window.</param>
        /// <param name="nCmdShow">A flag that determines how the window is displayed (e.g., SW_SHOW, SW_HIDE, SW_MAXIMIZE).</param>
        /// <returns>True if the window was previously visible, otherwise false.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool ShowWindow(IntPtr hWnd, int nCmdShow);

        /// <summary>
        /// Posts a message to the message loop to close the application.
        /// This causes a WM_QUIT message, after which the main message loop stops.
        /// </summary>
        /// <param name="nExitCode">The exit code returned to the operating system.</param>
        [LibraryImport(_user32, SetLastError = true)]
        internal static partial void PostQuitMessage(int nExitCode);

        /// <summary>
        /// Forces a window to repaint its client area by sending a WM_PAINT message.
        /// </summary>
        /// <param name="hWnd">The handle to the window that needs to be updated.</param>
        /// <returns>True if the update was successful, otherwise false.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool UpdateWindow(IntPtr hWnd);

        /// <summary>
        /// Retrieves system-related information and screen size, such as resolution or taskbar width.
        /// </summary>
        /// <param name="nIndex">The index of the system metric to retrieve (e.g., SM_CXSCREEN for screen width, SM_CYSCREEN for screen height).</param>
        /// <returns>The requested system value, or 0 on failure.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        internal static partial int GetSystemMetrics(int nIndex);

        /// <summary>
        /// Retrieves the Dimension of the client area of a window (excluding borders and title bar).
        /// </summary>
        /// <param name="hWnd">The handle to the window whose client size is requested.</param>
        /// <param name="lpRect">A RECT structure where the width and height of the client area will be stored.</param>
        /// <returns>True if the Dimension were successfully retrieved, otherwise false.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool GetClientRect(IntPtr hWnd, out RECT lpRect);

        /// <summary>
        /// Marks a portion of a window as invalid so that it will be redrawn in the next message loop.
        /// </summary>
        /// <param name="hWnd">The handle to the window that needs to be redrawn. Use IntPtr.Zero to invalidate all windows.</param>
        /// <param name="lpRect">A pointer to a RECT structure specifying the area to refresh, or IntPtr.Zero to invalidate the entire window.</param>
        /// <param name="bErase">True to erase the background before the window is redrawn, otherwise false.</param>
        /// <returns>True if invalidation was successful, otherwise false.</returns>
        [LibraryImport(_user32, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool InvalidateRect(
            IntPtr hWnd,
            IntPtr lpRect,
            [MarshalAs(UnmanagedType.Bool)] bool bErase
            );

        /// <summary>
        /// Retrieves a message from the message queue of a thread and waits for a message if necessary.
        /// </summary>
        /// <param name="lpMsg">A MSG structure that will be filled with message data.</param>
        /// <param name="hWnd">The handle to the window for which messages are retrieved, or IntPtr.Zero to retrieve messages for all windows in the thread.</param>
        /// <param name="wMsgFilterMin">The lowest message value that is accepted, or 0 to use no filter.</param>
        /// <param name="wMsgFilterMax">The highest message value that is accepted, or 0 to use no filter.</param>
        /// <returns>True if a message has been retrieved that needs to be processed, false if WM_QUIT is received.</returns>
        [LibraryImport(_user32, EntryPoint = "GetMessageW", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool GetMessage(
            out MSG lpMsg,
            IntPtr hWnd,
            uint wMsgFilterMin,
            uint wMsgFilterMax
            );

        /// <summary>
        /// Processes keyboard messages and converts virtual key codes to characters.
        /// </summary>
        /// <param name="lpMsg">A MSG structure containing the message to be translated.</param>
        /// <returns>True if the message was a key press and was translated, otherwise false.</returns>
        [LibraryImport(_user32, EntryPoint = "TranslateMessage", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        internal static partial bool TranslateMessage(ref MSG lpMsg);

        /// <summary>
        /// Sends a message to the appropriate window procedure (WndProc) for processing.
        /// </summary>
        /// <param name="lpMsg">A MSG structure containing the message to be delivered.</param>
        /// <returns>The return value from the window procedure.</returns>
        [LibraryImport(_user32, EntryPoint = "DispatchMessageW", SetLastError = true)]
        internal static partial IntPtr DispatchMessage(ref MSG lpMsg);

        /// <summary>
        /// Loads an image (such as an icon, cursor, or bitmap) from a resource file or file path.
        /// </summary>
        /// <param name="hInstance">The handle to the module containing the image. Use IntPtr.Zero to load a default system image.</param>
        /// <param name="lpszName">The name or ID of the image. This can be a file path or a resource ID as a string.</param>
        /// <param name="uType">The type of image to load (e.g., IMAGE_BITMAP, IMAGE_ICON, IMAGE_CURSOR).</param>
        /// <param name="cxDesired">The desired width of the image. Use 0 for the default size.</param>
        /// <param name="cyDesired">The desired height of the image. Use 0 for the default size.</param>
        /// <param name="fuLoad">Flags that determine how the image is loaded (e.g., LR_DEFAULTCOLOR, LR_LOADFROMFILE).</param>
        /// <returns>A handle to the loaded image, or IntPtr.Zero if loading failed.</returns>
        [LibraryImport(_user32, EntryPoint = "LoadImageW", SetLastError = true, StringMarshalling = StringMarshalling.Utf16)]
        internal static partial IntPtr LoadImage(IntPtr hInstance, string lpszName, uint uType, int cxDesired, int cyDesired, uint fuLoad);
    }
}
