using Color = Simulation.Util.Color;
using FontDescriptor = Simulation.Util.FontDescriptor;
using Label = Simulation.UserInterface.Label;
using LoadingScreen = Simulation.UserInterface.Components.LoadingScreen;
using SettingButton = Simulation.UserInterface.Components.SettingButton;
using SettingPanel = Simulation.UserInterface.Components.SettingPanel;
using TextStyles = Simulation.Util.TextStyles;
using UIEventComponent = Simulation.UserInterface.UIEventComponent;
using UnitStats = Simulation.UserInterface.Components.UnitStats;
using UnitToggleState = Simulation.UserInterface.Components.UnitToggleState;

namespace Simulation
{
    internal class UI : UIEventComponent
    {
        internal FontDescriptor Style { get; private set; }
        internal LoadingScreen Loadingscreen { get; private set; }
        internal SettingButton SettingButton { get; private set; }
        internal SettingPanel SettingPanel { get; private set; }
        internal Label CounterLabel { get; private set; }
        internal UnitToggleState UnitToggleState { get; private set; }
        internal UnitStats UnitStats { get; private set; }

        private const int _inputThresholdMs = 16;

        private DateTime _lastInputTime = DateTime.MinValue;

        private static UI? _instance;
        private static readonly object _lock = new();

        internal static UI Instance
        {
            get
            {
                if (_instance == null)
                    throw new InvalidOperationException("UI is not initialized, call the UI.Initialize() function.");

                return _instance;
            }
        }

        internal static void Initialize(
            string id = "UI",
            string prefix = "98_",
            string childPrefix = "99_")
        {
            lock (_lock)
                _instance ??= new UI(
                    id,
                    prefix,
                    childPrefix);
        }

        private UI(
            string id,
            string prefix,
            string childPrefix) : base($"{prefix}{id}")
        {
            Style = TextStyles.Readable;

            var files = Directory
                .GetFiles("Blueprint", "*.json")
                .Select(x => System.IO.Path.GetFileNameWithoutExtension(x))
                .ToArray();

            Loadingscreen = new(
                id: id,
                rectRadius: new(5.0f, 5.0f),
                style: Style,
                color: Color.Gray50,
                blueprints: files);

            SettingButton = new(
                id: id,
                style: Style,
                position: new(7.5f, 10f),
                padding: new(5, 0),
                textColor: Color.White,
                textHoverColor: Color.Cyan,
                buttonColor: Color.Black75,
                hoverColor: Color.Gray85);

            SettingPanel = new(
                id: $"{prefix}{id}",
                childID: $"{childPrefix}{id}",
                style: Style,
                panelPos: new(25, 60),
                panelColor: Color.Gray85,
                panelRounding: new(5, 5),
                padding: new(8, 8),
                elementSpacing: new(200, 17.5f),
                groupSpacing: new(0, 32.5f),
                defaultSlider: new(6.0f, 30.0f, 225.0f));

            CounterLabel = new(
                id: id,
                text: "Collision's:",
                style: Style,
                position: new(45f, 10f),
                padding: new(5, 0),
                textColor: Color.White,
                backgroundColor: Color.Black75,
                visible: false);

            UnitStats = new($"{childPrefix}{id}");
            UnitToggleState = new($"{childPrefix}{id}");

            SettingPanel.FPS.Slider.Interval = Cycle.TargetFPS;
            SettingPanel.UPS.Slider.Interval = Cycle.TargetUPS;
        }

        internal override void UpdateViewport(float scale)
        {
            Loadingscreen.UpdateViewport(scale);
            SettingButton.UpdateViewport(scale);
            SettingPanel.UpdateViewport(scale);
            UnitToggleState.UpdateViewport(scale);
            UnitStats.UpdateViewport(scale);
            Render();
        }

        internal void Refresh()
        {
            if (SettingPanel.Active)
            {
                if (SettingPanel.Counter.Active)
                    CounterLabel.Content = $"Collisions: {Environment.Instance.Collisions}";

                if (SettingPanel.Heatmap.Active)
                    Environment.Instance.RenderHeatmap();

                if (SettingPanel.Visible)
                {
                    if (SettingPanel.FPS.Slider.Clicked)
                        SettingPanel.FPS.Text.Content = $"{Cycle.ActorFPS} / {SettingPanel.FPS.Slider.Interval} FPS";
                    else
                        SettingPanel.FPS.Text.Content = Cycle.RenderText();

                    if (SettingPanel.UPS.Slider.Clicked)
                        SettingPanel.UPS.Text.Content = $"{Cycle.ActorUPS} / {SettingPanel.UPS.Slider.Interval} UPS";
                    else
                        SettingPanel.UPS.Text.Content = Cycle.UpdateText();
                }
                else if (UnitStats.Active)
                    UnitStats.Render();
            }
        }

        internal override void Render()
        {
            if (Loadingscreen.Visible)
            {
                Loadingscreen.Render();
                SettingPanel.Visible = false;
                return;
            }
            else if (!SettingButton.Visible)
                return;

            SettingButton.Render();

            if (SettingPanel.Visible)
            {
                UnitStats.Remove();
                SettingPanel.Render();
            }
            else
            {
                UnitToggleState.Render();
                UnitStats.Render();
            }
        }

        internal override void Reset()
        {
            Loadingscreen.Reset();
            SettingButton.Reset();
            SettingPanel.Reset();
            CounterLabel.Reset();
            UnitToggleState.Reset();
            UnitStats.Reset();
        }

        internal override void Remove()
        {
            Loadingscreen.Remove();
            SettingButton.Remove();
            SettingPanel.Remove();
            CounterLabel.Remove();
            UnitToggleState.Remove();
            UnitStats.Remove();
        }

        internal override bool LeftClick(float X, float Y)
        {
            if ((DateTime.UtcNow - _lastInputTime).TotalMilliseconds < _inputThresholdMs)
                return false;

            _lastInputTime = DateTime.UtcNow;

            if (Loadingscreen.Visible)
            {
                return Loadingscreen.LeftClick(X, Y);
            }
            else if (!SettingButton.Visible)
                return false;

            if (SettingButton.LeftClick(X, Y))
                return true;

            if (SettingPanel.Visible)
            {
                return SettingPanel.LeftClick(X, Y);
            }
            else
            {
                return UnitStats.LeftClick(X, Y);
            }
        }

        internal override bool LeftRelease()
        {
            _lastInputTime = DateTime.UtcNow;

            if (Loadingscreen.Visible)
            {
                return Loadingscreen.LeftRelease();
            }
            else if (!SettingButton.Visible)
                return false;

            if (SettingButton.LeftRelease())
                return true;

            if (SettingPanel.Visible)
            {
                return SettingPanel.LeftRelease();
            }
            else
            {
                return UnitStats.LeftRelease();
            }
        }

        internal override bool RightClick(float X, float Y)
        {
            if ((DateTime.UtcNow - _lastInputTime).TotalMilliseconds < _inputThresholdMs)
                return false;

            _lastInputTime = DateTime.UtcNow;

            if (Loadingscreen.Visible)
                return false;
            else if (!SettingButton.Visible)
                return false;

            if (!SettingPanel.Visible)
                return UnitToggleState.RightClick(X, Y);

            return false;
        }

        internal override bool RightRelease()
        {
            _lastInputTime = DateTime.UtcNow;

            if (Loadingscreen.Active)
                return false;
            else if (!SettingButton.Visible)
                return false;

            if (!SettingPanel.Visible)
                return UnitToggleState.RightRelease();

            return false;
        }

        internal override void Hover(float X, float Y)
        {
            if (Loadingscreen.Active)
            {
                Loadingscreen.Hover(X, Y);
                return;
            }
            else
            {
                SettingButton.Hover(X, Y);
            }

            if (SettingPanel.Visible)
            {
                SettingPanel.Hover(X, Y);
            }
            else
            {
                UnitToggleState.Hover(X, Y);
                UnitStats.Hover(X, Y);
            }
        }
    }
}
