using Interaction = Win32.Interaction;
using MouseButton = Win32.MouseButton;
using VirtualKey = Win32.VirtualKey;
using Window = Win32.Window;

namespace Simulation
{
    internal class Windower
    {
        private readonly Window _window;
        private static Windower? _instance;
        private static readonly object _lock = new();

        internal static Windower Instance
        {
            get
            {
                if (_instance == null)
                    throw new InvalidOperationException("Renderer is not initialized, call the Renderer.Initialize() function.");

                return _instance;
            }
        }

        internal static void Initialize()
        {
            lock (_lock)
                _instance ??= new Windower();
        }

        private Windower(string window = "Simulation", string title = "i4sim", string iconPath = "Resource/icon.ico")
        {
            _window = new(window, title, iconPath);
        }

        internal nint GetHandler()
        {
            return _window.HWnd;
        }

        internal (int Left, int Right, int Top, int Bottom) GetClientRect()
        {
            return _window.GetClientRect();
        }

        internal void RegisterEvents()
        {
            _window.SizeEvent += (sender, e) =>
            {
                Renderer.Instance.UpdateViewport(e.Width, e.Height);
            };

            _window.MouseEvent += (sender, e) =>
            {
                switch (e.Interaction)
                {
                    case Interaction.Press:
                        switch (e.Button)
                        {
                            case MouseButton.Left:
                                UI.Instance.LeftClick(e.X, e.Y);
                                break;

                            case MouseButton.Right:
                                UI.Instance.RightClick(e.X, e.Y);
                                break;
                        }
                        break;

                    case Interaction.Release:
                        switch (e.Button)
                        {
                            case MouseButton.Left:
                                UI.Instance.LeftRelease();
                                break;
                            case MouseButton.Right:
                                UI.Instance.RightRelease();
                                break;
                        }
                        break;

                    case Interaction.Move:
                        UI.Instance.Hover(e.X, e.Y);
                        break;
                }
            };

            _window.KeyEvent += (sender, e) =>
            {
                if (e.Interaction == Interaction.Press)
                    switch (e.KeyCode)
                    {
                        case VirtualKey.ESCAPE:
                            App.Quit();
                            break;
                        case VirtualKey.CONTROL:
                            if (Cycle.IsRunning)
                                UI.Instance.SettingButton.Interact();
                            break;
                        case VirtualKey.SPACE:
                            if (Cycle.IsRunning)
                            {
                                Cycle.Toggle();
                                UI.Instance.SettingPanel.Pause.Interact();
                            }
                            break;
                    }
            };

            _window.DestroyEvent += (sender, e) =>
            {
                App.Quit();
            };
        }

        internal void Run()
        {
            _window.Run();
        }

        internal void Refresh()
        {
            _window.Refresh();
        }
    }
}
