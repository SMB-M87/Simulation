using BorderSlider = Simulation.UserInterface.Components.Settings.BorderSlider;
using Colors = Simulation.Util.Color;
using CounterSlider = Simulation.UserInterface.Components.Settings.CounterSlider;
using DebugFlag = Simulation.UnitTransport.DebugFlag;
using DebugSlider = Simulation.UserInterface.Components.Settings.DebugSlider;
using DummySlider = Simulation.UserInterface.Components.Settings.DummySlider;
using FontDescriptor = Simulation.Util.FontDescriptor;
using FPSSlider = Simulation.UserInterface.Components.Settings.FPSSlider;
using GridSlider = Simulation.UserInterface.Components.Settings.GridSlider;
using HeatmapSlider = Simulation.UserInterface.Components.Settings.HeatmapSlider;
using LoadscreenSlider = Simulation.UserInterface.Components.Settings.LoadscreenSlider;
using MQTTSlider = Simulation.UserInterface.Components.Settings.MQTTSlider;
using PauseSlider = Simulation.UserInterface.Components.Settings.PauseSlider;
using UPSSlider = Simulation.UserInterface.Components.Settings.UPSSlider;
using Vector2 = System.Numerics.Vector2;
using Vector3 = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components
{
    internal class SettingPanel(
        string id,
        string childID,
        FontDescriptor style,
        Vector2 panelPos,
        Vector4 panelColor,
        Vector2 panelRounding,
        Vector2 padding,
        Vector2 elementSpacing,
        Vector2 groupSpacing,
        Vector3 defaultSlider
        )
        : Panel(
            $"{id}_setting",
            panelPos,
            new(padding.X * 2 + defaultSlider.Z + defaultSlider.X * 2,
                Math.Abs(panelPos.Y - elementSpacing.Y) + (35 * elementSpacing.Y) + (3 * groupSpacing.Y) + padding.Y * 3),
            panelRounding,
            panelColor,
            panelColor
        )
    {
        private readonly string _childID = childID;

        internal override bool LeftClick(float X, float Y)
        {
            if (!base.LeftClick(X, Y)) return false;

            if (Loadscreen.LeftClick(X, Y)) return true;
            if (Pause.LeftClick(X, Y)) return true;
            if (MQTT.LeftClick(X, Y)) return true;
            if (Dummy.LeftClick(X, Y)) return true;

            if (Counter.LeftClick(X, Y)) return true;
            if (LogMQTT.LeftClick(X, Y)) return true;
            if (LogProducts.LeftClick(X, Y)) return true;
            if (LogMovers.LeftClick(X, Y)) return true;

            if (Grid.LeftClick(X, Y)) return true;
            if (Border.LeftClick(X, Y)) return true;
            if (Velocity.LeftClick(X, Y)) return true;
            if (Acceleration.LeftClick(X, Y)) return true;
            if (Radius.LeftClick(X, Y)) return true;
            if (Detection.LeftClick(X, Y)) return true;
            if (Path.LeftClick(X, Y)) return true;
            if (Heatmap.LeftClick(X, Y)) return true;

            if (FPS.LeftClick(X, Y)) return true;
            if (UPS.LeftClick(X, Y)) return true;

            return false;
        }

        internal override bool LeftRelease()
        {
            if (!base.LeftRelease()) return false;

            if (Loadscreen.LeftRelease()) return true;
            if (Pause.LeftRelease()) return true;
            if (MQTT.LeftRelease()) return true;
            if (Dummy.LeftRelease()) return true;

            if (Counter.LeftRelease()) return true;
            if (LogMQTT.LeftRelease()) return true;
            if (LogProducts.LeftRelease()) return true;
            if (LogMovers.LeftRelease()) return true;

            if (Grid.LeftRelease()) return true;
            if (Border.LeftRelease()) return true;
            if (Velocity.LeftRelease()) return true;
            if (Acceleration.LeftRelease()) return true;
            if (Radius.LeftRelease()) return true;
            if (Detection.LeftRelease()) return true;
            if (Path.LeftRelease()) return true;
            if (Heatmap.LeftRelease()) return true;

            if (FPS.LeftRelease()) return true;
            if (UPS.LeftRelease()) return true;

            return false;
        }

        internal override void Hover(float X, float Y)
        {
            base.Hover(X, Y);

            Loadscreen.Hover(X, Y);
            Pause.Hover(X, Y);
            MQTT.Hover(X, Y);
            Dummy.Hover(X, Y);

            Counter.Hover(X, Y);
            LogMQTT.Hover(X, Y);
            LogProducts.Hover(X, Y);
            LogMovers.Hover(X, Y);

            Grid.Hover(X, Y);
            Border.Hover(X, Y);
            Velocity.Hover(X, Y);
            Acceleration.Hover(X, Y);
            Radius.Hover(X, Y);
            Detection.Hover(X, Y);
            Path.Hover(X, Y);
            Heatmap.Hover(X, Y);

            FPS.Hover(X, Y);
            UPS.Hover(X, Y);
        }

        internal override void UpdateViewport(float scale)
        {
            base.UpdateViewport(scale);

            Loadscreen.UpdateViewport(scale);
            Pause.UpdateViewport(scale);
            MQTT.UpdateViewport(scale);
            Dummy.UpdateViewport(scale);

            Counter.UpdateViewport(scale);
            LogMQTT.UpdateViewport(scale);
            LogProducts.UpdateViewport(scale);
            LogMovers.UpdateViewport(scale);

            Grid.UpdateViewport(scale);
            Border.UpdateViewport(scale);
            Velocity.UpdateViewport(scale);
            Acceleration.UpdateViewport(scale);
            Radius.UpdateViewport(scale);
            Detection.UpdateViewport(scale);
            Path.UpdateViewport(scale);
            Heatmap.UpdateViewport(scale);

            FPS.UpdateViewport(scale);
            UPS.UpdateViewport(scale);
        }

        internal override void Render()
        {
            base.Render();

            Loadscreen.Render();
            Pause.Render();
            MQTT.Render();
            Dummy.Render();

            Counter.Render();
            LogMQTT.Render();
            LogProducts.Render();
            LogMovers.Render();

            Grid.Render();
            Border.Render();
            Velocity.Render();
            Acceleration.Render();
            Radius.Render();
            Detection.Render();
            Path.Render();
            Heatmap.Render();

            FPS.Render();
            UPS.Render();
        }

        internal override void Reset()
        {
            base.Reset();

            Loadscreen.Reset();
            Pause.Reset();
            MQTT.Reset();
            Dummy.Reset();

            Counter.Reset();
            LogMQTT.Reset();
            LogProducts.Reset();
            LogMovers.Reset();

            Grid.Reset();
            Border.Reset();
            Velocity.Reset();
            Acceleration.Reset();
            Radius.Reset();
            Detection.Reset();
            Path.Reset();
            Heatmap.Reset();
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand(_childID);
            base.Remove();
        }

        internal SliderToggleText Loadscreen { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Loadscreen",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new LoadscreenSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    false));

        internal SliderToggleText Pause { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Paused",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new PauseSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 3),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText MQTT { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "MQTT",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 4),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new MQTTSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 5),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    false));

        internal SliderToggleText Dummy { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Dummy",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 6),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DummySlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 7),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    false));

        internal SliderToggleText Counter { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Counter",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 8 + groupSpacing.Y * 1),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new CounterSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 9 + groupSpacing.Y * 1),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText LogMQTT { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Log MQTT",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 10 + groupSpacing.Y * 1),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new SliderToggle(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 11 + groupSpacing.Y * 1),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText LogProducts { get; private set; }
            = new(childID,

                new Text(
                    childID,
                    "Log Products",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 12 + groupSpacing.Y * 1),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new SliderToggle(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 13 + groupSpacing.Y * 1),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText LogMovers { get; private set; }
            = new(childID,

                new Text(
                    childID,
                    "Log Movers",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 14 + groupSpacing.Y * 1),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new SliderToggle(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 15 + groupSpacing.Y * 1),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText Grid { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Grid",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 16 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new GridSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 17 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText Border { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Border",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 18 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new BorderSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 19 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    true));

        internal SliderToggleText Velocity { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Velocity",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 20 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DebugSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 21 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    DebugFlag.Velocity,
                    false,
                    false));

        internal SliderToggleText Acceleration { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Acceleration",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 22 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DebugSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 23 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    DebugFlag.Acceleration,
                    false,
                    false));

        internal SliderToggleText Radius { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Radius",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 24 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DebugSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 25 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    DebugFlag.Radius,
                    false,
                    false));

        internal SliderToggleText Detection { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Detection",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 26 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DebugSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 27 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    DebugFlag.Detection,
                    false,
                    false));


        internal SliderToggleText Path { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Path",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 28 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new DebugSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 29 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    DebugFlag.Path,
                    false,
                    false));

        internal SliderToggleText Heatmap { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "Heatmap",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 30 + groupSpacing.Y * 2),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new HeatmapSlider(
                    childID,
                    new(panelPos.X + padding.X + elementSpacing.X, panelPos.Y + padding.Y + elementSpacing.Y * 31 + groupSpacing.Y * 2),
                    new(defaultSlider.X, defaultSlider.Y),
                    Colors.White75,
                    Colors.White,
                    false,
                    false));

        internal SliderText FPS { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "FPS",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 32 + groupSpacing.Y * 3),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new FPSSlider(
                    childID,
                    new(panelPos.X + padding.X + defaultSlider.X, panelPos.Y + padding.Y + elementSpacing.Y * 34 + groupSpacing.Y * 3 + defaultSlider.X),
                    new(defaultSlider.X, defaultSlider.Z),
                    Colors.White75,
                    Colors.White));

        internal SliderText UPS { get; private set; } =
            new(childID,

                new Text(
                    childID,
                    "UPS",
                    style,
                    new(panelPos.X + padding.X, panelPos.Y + padding.Y + elementSpacing.Y * 35 + groupSpacing.Y * 3),
                    new(0, 0),
                    Colors.White,
                    Colors.White75),

                new UPSSlider(
                    childID,
                    new(panelPos.X + padding.X + defaultSlider.X, panelPos.Y + padding.Y + elementSpacing.Y * 37 + groupSpacing.Y * 3 + defaultSlider.X),
                    new(defaultSlider.X, defaultSlider.Z),
                    Colors.White75,
                    Colors.White));
    }
}
