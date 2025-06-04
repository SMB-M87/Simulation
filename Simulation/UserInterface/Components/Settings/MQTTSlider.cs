using Client = Simulation.MQTT.Client;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components.Settings
{
    internal class MQTTSlider(
        string id,
        Vector2 position,
        Vector2 slider,
        Vector4 barColor,
        Vector4 ballColor,
        bool visible = false,
        bool active = false
        ) : SliderToggle(
            id,
            position,
            slider,
            barColor,
            ballColor,
            visible,
            active
            )
    {
        internal override void OnLeftClick()
        {
            if (UI.Instance.SettingPanel.Dummy.Active)
            {
                Reset();
                return;
            }

            base.OnLeftClick();

            if (Active)
            {
                Client.Instance.Start();
            }
            else
            {
                Client.Instance.Stop();
            }
        }

        internal override void Interact()
        {
            if (UI.Instance.SettingPanel.Dummy.Active)
            {
                return;
            }

            UI.Instance.SettingPanel.Dummy.Active = false;
        }

        internal override void Reset()
        {
            if (Active)
                Client.Instance.Stop();

            base.Reset();
        }
    }
}
