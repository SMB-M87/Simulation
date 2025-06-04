using Procedure = Simulation.Dummy.Procedure;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components.Settings
{
    internal class DummySlider(
        string id,
        Vector2 position,
        Vector2 slider,
        Vector4 barColor,
        Vector4 ballColor,
        bool visible = false,
        bool active = false
        ) : SliderToggle(
            id, position,
            slider,
            barColor,
            ballColor,
            visible,
            active)
    {
        internal override void OnLeftClick()
        {
            if (UI.Instance.SettingPanel.MQTT.Active)
            {
                Reset();
                return;
            }

            base.OnLeftClick();

            if (Active)
            {
                Procedure.Instance.Start();
            }
            else
            {
                Procedure.Instance.Stop();
            }
        }

        internal override void Reset()
        {
            if (Active)
                Procedure.Instance.Stop();

            base.Reset();
        }
    }
}
