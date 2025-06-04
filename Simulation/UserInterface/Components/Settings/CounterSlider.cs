using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components.Settings
{
    internal class CounterSlider(
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
            active
            )
    {
        internal override void OnLeftClick()
        {
            base.OnLeftClick();

            if (Active)
            {
                UI.Instance.CounterLabel.Render();
            }
            else
            {
                UI.Instance.CounterLabel.Remove();
            }
        }
    }
}
