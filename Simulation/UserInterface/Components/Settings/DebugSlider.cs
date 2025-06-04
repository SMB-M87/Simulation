using DebugFlag = Simulation.UnitTransport.DebugFlag;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components.Settings
{
    internal class DebugSlider(
        string id,
        Vector2 position,
        Vector2 slider,
        Vector4 barColor,
        Vector4 ballColor,
        DebugFlag debug,
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
        private readonly DebugFlag _debug = debug;

        internal override void OnLeftClick()
        {
            base.OnLeftClick();

            foreach (var mover in Environment.Instance.Movers.Get())
                mover.ChangeDebugView(_debug);
        }
    }
}
