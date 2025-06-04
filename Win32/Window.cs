using DestroyEvent = Win32.Event.DestroyEvent;
using DestroyHandler = Win32.Handler.DestroyHandler;
using GCHandle = System.Runtime.InteropServices.GCHandle;
using IWindowHandler = Win32.Handler.IWindowHandler;
using KeyEvent = Win32.Event.KeyEvent;
using KeyHandler = Win32.Handler.KeyHandler;
using Marshal = System.Runtime.InteropServices.Marshal;
using MouseEvent = Win32.Event.MouseEvent;
using MouseHandler = Win32.Handler.MouseHandler;
using RenderHandler = Win32.Handler.RenderHandler;
using SizeEvent = Win32.Event.SizeEvent;
using SizeHandler = Win32.Handler.SizeHandler;
using WNDCLASSEX = Win32.Struct.WNDCLASSEX;

namespace Win32
{
    /// <summary>
    /// Manages the creation, registration, and lifecycle of a native Win32 application window.
    /// 
    /// Responsibilities:
    /// <list type="bullet">
    ///   <item><description><b>Window creation:</b> Registers a custom window class and creates a full-screen overlapped window.</description></item>
    ///   <item><description><b>Rendering setup:</b> Initializes Direct2D for hardware-accelerated drawing using the client area.</description></item>
    ///   <item><description><b>Event dispatching:</b> Routes Windows messages (resize, mouse, keyboard, paint, destroy) to internal handlers via <c>WndProc</c>.</description></item>
    ///   <item><description><b>Input handling:</b> Exposes typed events (<see cref="SizeEvent"/>, <see cref="MouseEvent"/>, <see cref="KeyEvent"/>) to consumer code.</description></item>
    ///   <item><description><b>Application loop:</b> Starts and runs the main message loop, processing all pending Windows messages until shutdown.</description></item>
    /// </list>
    /// 
    /// Relevant Win32 references:
    /// <list type="bullet">
    ///   <item><description><see href="https://learn.microsoft.com/en-us/windows/win32/learnwin32/creating-a-window">Creating a Window</see></description></item>
    ///   <item><description><see href="https://learn.microsoft.com/en-us/windows/win32/winmsg/window-classes">Window Classes</see></description></item>
    ///   <item><description><see href="https://learn.microsoft.com/en-us/windows/win32/winmsg/about-messages-and-message-queues">Message Queues</see></description></item>
    /// </list>
    /// </summary>
    public sealed partial class Window
    {
        /// <summary>
        /// Native handle to the created Win32 window (HWND).
        /// Used for message dispatch, rendering, and input binding.
        /// </summary>
        public nint HWnd { get; private set; }

        /// <summary>
        /// Event triggered when the window is resized (WM_SIZE).
        /// </summary>
        public event EventHandler<SizeEvent>? SizeEvent;

        /// <summary>
        /// Event triggered on mouse input (WM_LBUTTONDOWN, WM_LBUTTONUP, WM_MOUSEMOVE).
        /// </summary>
        public event EventHandler<MouseEvent>? MouseEvent;

        /// <summary>
        /// Event triggered on key input (WM_KEYDOWN, WM_KEYUP).
        /// </summary>
        public event EventHandler<KeyEvent>? KeyEvent;

        /// <summary>
        /// Event triggered when the window is destroyed (WM_DESTROY).
        /// </summary>
        public event EventHandler<DestroyEvent>? DestroyEvent;

        /// <summary>
        /// Win32 style constant for creating a standard overlapped window,
        /// including title bar, border, and minimize/maximize/close buttons.
        /// </summary>
        private const int _ws_OVERLAPPEDWINDOW = 0x00CF0000;

        /// <summary>
        /// Win32 system metric index for retrieving the width of the primary screen in pixels.
        /// </summary>
        private const int _sm_CXSCREEN = 0;

        /// <summary>
        /// Win32 system metric index for retrieving the height of the primary screen in pixels.
        /// </summary>
        private const int _sm_CYSCREEN = 1;

        /// <summary>
        /// Index used with GetWindowLongPtr/SetWindowLongPtr to store custom user data
        /// (e.g., a GCHandle to the <c>Window</c> object) in the window instance.
        /// </summary>
        private const int _gwl_USERDATA = -21;

        /// <summary>
        /// Win32 constant for the default arrow cursor (IDC_ARROW),
        /// used to display a pointer cursor for the window.
        /// </summary>
        private const int _idc_ARROW = 32512;

        /// <summary>
        /// An instance of <see cref="MouseHandler"/> that processes mouse input events.
        /// <list type="bullet">
        ///   <item><description>Handles mouse presses, releases, and movement.</description></item>
        ///   <item><description>Supports interactive UI elements such as sliders or buttons.</description></item>
        ///   <item><description>Registered under multiple message codes in <c>_handlers</c> to centralize logic.</description></item>
        /// </list>
        /// </summary>
        private static readonly MouseHandler _mouseHandler = new();

        /// <summary>
        /// A dictionary that maps Windows message codes to their corresponding message handlers.
        /// <list type="bullet">
        ///   <item><description><b>WM_PAINT:</b> Handled by <see cref="RenderHandler"/> to perform rendering.</description></item>
        ///   <item><description><b>WM_KEYDOWN:</b> Handled by <see cref="KeyHandler"/> to process key presses.</description></item>
        ///   <item><description><b>Mouse input:</b> <see cref="_mouseHandler"/> handles down, up, and move events.</description></item>
        ///   <item><description><b>WM_SIZE:</b> Triggers <see cref="SizeHandler"/> to handle window resizing.</description></item>
        ///   <item><description><b>WM_DESTROY:</b> Uses <see cref="DestroyHandler"/> to clean up and exit the app.</description></item>
        /// </list>
        /// </summary>
        private static readonly Dictionary<uint, IWindowHandler> _handlers = new()
        {
            { MessageCode.WM_PAINT, new RenderHandler() },
            { MessageCode.WM_KEYDOWN, new KeyHandler() },
            { MessageCode.WM_LBUTTONDOWN, _mouseHandler },
            { MessageCode.WM_LBUTTONUP, _mouseHandler },
            { MessageCode.WM_RBUTTONDOWN, _mouseHandler },
            { MessageCode.WM_RBUTTONUP, _mouseHandler },
            { MessageCode.WM_MOUSEMOVE, _mouseHandler },
            { MessageCode.WM_SIZE, new SizeHandler() },
            { MessageCode.WM_DESTROY, new DestroyHandler(_gwl_USERDATA) }
        };

        /// <summary>
        /// Constant used to display a window using the Win32 <c>ShowWindow</c> API.
        /// <list type="bullet">
        ///   <item><description>Represents the <c>SW_SHOW</c> flag with value <c>5</c>.</description></item>
        ///   <item><description>Ensures the window is activated and displayed in its current size and position.</description></item>
        ///   <item><description>Used during application startup to make the window visible on screen.</description></item>
        /// </list>
        /// </summary>
        private const int _sw_SHOW = 5;

        /// <summary>
        /// Invokes the <see cref="SizeEvent"/> to notify listeners of a window resize.
        /// <list type="bullet">
        ///   <item><description>Provides updated width and height information to any subscribers.</description></item>
        ///   <item><description>Typically used to update rendering or UI layout in response to resolution changes.</description></item>
        /// </list>
        /// </summary>
        /// <param name="e">The event data containing the new window size.</param>
        internal void OnSizeEvent(SizeEvent e)
        {
            SizeEvent?.Invoke(this, e);
        }

        /// <summary>
        /// Invokes the <see cref="MouseEvent"/> to report a mouse input event.
        /// <list type="bullet">
        ///   <item><description>Includes position and interaction type (e.g., move, press, release).</description></item>
        ///   <item><description>Used for triggering UI interactions or in-scene input responses.</description></item>
        /// </list>
        /// </summary>
        /// <param name="e">The event data containing mouse state information.</param>
        internal void OnMouseEvent(MouseEvent e)
        {
            MouseEvent?.Invoke(this, e);
        }

        /// <summary>
        /// Invokes the <see cref="KeyEvent"/> to report a keyboard input event.
        /// <list type="bullet">
        ///   <item><description>Distinguishes between key presses and releases.</description></item>
        ///   <item><description>Used to trigger simulation control actions such as pause, quit, or UI toggling.</description></item>
        /// </list>
        /// </summary>
        /// <param name="e">The event data containing key press or release information.</param>
        internal void OnKeyEvent(KeyEvent e)
        {
            KeyEvent?.Invoke(this, e);
        }

        /// <summary>
        /// Invokes the <see cref="DestroyEvent"/> to notify listeners of window destruction.
        /// <list type="bullet">
        ///   <item><description>Ensures clean shutdown and resource release.</description></item>
        ///   <item><description>Triggers any external subscribers to handle application termination.</description></item>
        /// </list>
        /// </summary>
        /// <param name="e">The event data describing the destruction event.</param>
        internal void OnDestroyEvent(DestroyEvent e)
        {
            DestroyEvent?.Invoke(this, e);
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Window"/> class:
        /// <list type="bullet">
        ///   <item><description>Registers a new window class using the provided name and optional icon.</description></item>
        ///   <item><description>Creates a full-screen overlapped window using Win32 APIs.</description></item>
        ///   <item><description>Binds this <c>Window</c> instance to the native HWND for message routing.</description></item>
        ///   <item><description>Initializes Direct2D rendering using the window’s client dimensions.</description></item>
        /// </list>
        /// Throws an <see cref="Exception"/> if the window class cannot be registered or the window cannot be created.
        /// </summary>
        /// <param name="window">The class name to register the window under.</param>
        /// <param name="title">The window title shown in the title bar.</param>
        /// <param name="iconPath">Optional path to a window icon (e.g., <c>"Resource/icon.ico"</c>).</param>
        public Window(string window, string title, string iconPath = "")
        {
            if (WindowRegister(window, iconPath) == 0)
            {
                var errorCode = Marshal.GetLastPInvokeError();
                throw new Exception($"Window Class registration failed! Windows Error Code: {errorCode}");
            }

            HWnd = Interop.CreateWindowEx(
                0,
                window,
                title,
                _ws_OVERLAPPEDWINDOW,
                0, 0,
                Interop.GetSystemMetrics(_sm_CXSCREEN),
                Interop.GetSystemMetrics(_sm_CYSCREEN),
                nint.Zero,
                nint.Zero,
                nint.Zero,
                nint.Zero
                );

            if (HWnd == nint.Zero)
            {
                var errorCode = Marshal.GetLastPInvokeError();
                throw new Exception($"Window couldn't be generated! Windows Error Code: {errorCode}");
            }

            Interop.SetWindowLongPtr(HWnd, _gwl_USERDATA, GCHandle.ToIntPtr(GCHandle.Alloc(this)));
            Interop.ShowCursor(true);
        }

        /// <summary>
        /// Registers a window class with the operating system, enabling window creation and message routing.
        /// <list type="bullet">
        ///   <item><description>Defines the window procedure, icons, cursor, and background style.</description></item>
        ///   <item><description>Allocates native memory for the window class structure and registers it with Win32.</description></item>
        /// </list>
        /// Returns a class ID (ATOM) if successful; returns 0 on failure.
        /// <b>Throws an <see cref="Exception"/> if registration fails, including the native Windows error code.</b>
        /// </summary>
        /// <param name="window">The class name used to identify the registered window class.</param>
        /// <param name="iconPath">Optional file path to a window icon (used for large and small display).</param>
        /// <returns>A class ID (ATOM) if successful, or 0 if the call fails.</returns>
        /// <exception cref="Exception">
        /// Raised when the Windows API fails to register the window class. Includes the Windows error code.
        /// </exception>
        private static ushort WindowRegister(string window, string iconPath)
        {
            var classNamePtr = Marshal.StringToHGlobalUni(window);
            var menuNamePtr = nint.Zero;
            var hInstance = Interop.GetModuleHandle(null);
            var hIcon = Interop.LoadImage(
                hInstance,
                @iconPath,
                Interop._image_ICON,
                32, 32,
                Interop._lr_LOADFROMFILE
                );
            var hIconSm = Interop.LoadImage(
                hInstance,
                @iconPath,
                Interop._image_ICON,
                16, 16,
                Interop._lr_LOADFROMFILE
                );

            WNDCLASSEX wc = new()
            {
                cbSize = (uint)Marshal.SizeOf(typeof(WNDCLASSEX)),
                lpfnWndProc = Marshal.GetFunctionPointerForDelegate((Interop.WndProc)WindowProc),
                hInstance = hInstance,
                hCursor = Interop.LoadCursor(nint.Zero, _idc_ARROW),
                hbrBackground = 1,
                lpszClassName = classNamePtr,
                lpszMenuName = menuNamePtr,
                hIcon = hIcon,
                hIconSm = hIconSm
            };

            var wcPtr = Marshal.AllocHGlobal(Marshal.SizeOf<WNDCLASSEX>());
            Marshal.StructureToPtr(wc, wcPtr, false);
            var result = Interop.RegisterClassEx(wcPtr);
            Marshal.FreeHGlobal(wcPtr);
            Marshal.FreeHGlobal(classNamePtr);

            return result;
        }

        /// <summary>
        /// The window procedure (WndProc) that dispatches incoming Windows messages.
        /// <list type="bullet">
        ///   <item><description>Retrieves the associated <see cref="Window"/> instance using user data.</description></item>
        ///   <item><description>If the instance is available, delegates the message to <see cref="ProcessMessage"/>.</description></item>
        ///   <item><description>If not, forwards the message to <see cref="Interop.DefWindowProc"/> for default handling.</description></item>
        /// </list>
        /// </summary>
        /// <param name="hWnd">The handle of the window receiving the message.</param>
        /// <param name="msg">The message code indicating the type of event.</param>
        /// <param name="wParam">Message-specific parameter, varies by message type.</param>
        /// <param name="lParam">Message-specific parameter, varies by message type.</param>
        /// <returns>The result of message processing, or the result of the default window procedure.</returns>
        private static nint WindowProc(nint hWnd, uint msg, nint wParam, nint lParam)
        {
            var ptr = Interop.GetWindowLongPtr(hWnd, _gwl_USERDATA);

            if (ptr == nint.Zero)
                return Interop.DefWindowProc(hWnd, msg, wParam, lParam);

            var windowInstance = (Window)GCHandle.FromIntPtr(ptr).Target!;
            return windowInstance.ProcessMessage(msg, wParam, lParam);
        }

        /// <summary>
        /// Processes a single Windows message using a registered handler if available.
        /// <list type="bullet">
        ///   <item><description>Checks the <see cref="_handlers"/> dictionary for a handler matching the message ID.</description></item>
        ///   <item><description>Delegates to the appropriate <see cref="IWindowHandler"/> if found.</description></item>
        ///   <item><description>If no handler exists, falls back to <see cref="Interop.DefWindowProc"/> for default message handling.</description></item>
        /// </list>
        /// </summary>
        /// <param name="msg">The message identifier (e.g., WM_SIZE, WM_PAINT).</param>
        /// <param name="wParam">Additional message-specific information.</param>
        /// <param name="lParam">Additional message-specific information.</param>
        /// <returns>
        /// A pointer (nint) representing the result of the message handling,
        /// or the return value of <see cref="Interop.DefWindowProc"/> if unhandled.
        /// </returns>
        private nint ProcessMessage(uint msg, nint wParam, nint lParam)
        {
            if (_handlers.TryGetValue(msg, out var handler))
                return handler.HandleMessage(this, msg, wParam, lParam);

            return Interop.DefWindowProc(HWnd, msg, wParam, lParam);
        }

        /// <summary>
        /// Starts the window’s message loop and enters the main simulation runtime.
        /// <list type="bullet">
        ///   <item><description>Shows and updates the window using the Win32 API.</description></item>
        ///   <item><description>Enters a blocking loop that continuously processes Windows messages via <c>GetMessage</c>.</description></item>
        ///   <item><description>Calls <c>TranslateMessage</c> and <c>DispatchMessage</c> for standard message handling.</description></item>
        /// </list>
        /// </summary>
        public void Run()
        {
            Interop.ShowWindow(HWnd, _sw_SHOW);
            Interop.UpdateWindow(HWnd);

            while (Interop.GetMessage(out var msg, nint.Zero, 0, 0))
            {
                Interop.TranslateMessage(ref msg);
                Interop.DispatchMessage(ref msg);
            }
        }

        /// <summary>
        /// Get the dimensions of the client area of the window.
        /// </summary>
        /// <returns>Returns the left, right, top and bottom coordinates.</returns>
        public (int Left, int Right, int Top, int Bottom) GetClientRect()
        {
            Interop.GetClientRect(HWnd, out var rect);
            return (rect.left, rect.right, rect.top, rect.bottom);
        }

        /// <summary>
        /// Used to invalidate the window and refresh the window content.
        /// </summary>
        public void Refresh()
        {
            Interop.InvalidateRect(HWnd, IntPtr.Zero, false);
        }
    }
}