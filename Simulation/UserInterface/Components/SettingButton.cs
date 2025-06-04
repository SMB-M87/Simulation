using FontDescriptor = Simulation.Util.FontDescriptor;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface.Components
{
    /// <summary>
    /// A UI button component that wraps text with padding and renders a rounded rectangle background.
    /// </summary>
    /// <param name="id">Unique ID used for rendering draw commands.</param>
    /// <param name="text">Initial text content of the button.</param>
    /// <param name="style">Text style definition (font family, size, weight).</param>
    /// <param name="position">Base position of the button before scaling.</param>
    /// <param name="padding">Padding around the text (horizontal and vertical) before scaling.</param>
    /// <param name="textColor">Color of the button text.</param>
    /// <param name="buttonColor">Background color of the button.</param>
    /// <param name="hoverColor">Background color of the button.</param>
    internal class SettingButton(
        string id,
        FontDescriptor style,
        Vector2 position,
        Vector2 padding,
        Vector4 textColor,
        Vector4 textHoverColor,
        Vector4 buttonColor,
        Vector4 hoverColor,
        bool active = false
        ) : Button
        (
            $"{id}_setting",
            active ? "<<" : ">>",
            style,
            position,
            padding,
            textColor,
            textHoverColor,
            buttonColor,
            hoverColor,
            active
        )
    {
        internal override void OnLeftClick()
        {
            base.OnLeftClick();

            Active = !Active;
            Text = Active ? "<<" : ">>";
            base.Render();
            UI.Instance.SettingPanel.Visible = Active;

            if (UI.Instance.SettingPanel.Visible == true)
            {
                UI.Instance.UnitStats.Remove();
                UI.Instance.SettingPanel.Render();
                return;
            }
            else
            {
                UI.Instance.SettingPanel.Remove();
                UI.Instance.UnitStats.Render();
                return;
            }
        }

        internal override void Interact()
        {
            Active = !Active;
            Text = Active ? "<<" : ">>";
            base.Render();
            UI.Instance.SettingPanel.Visible = Active;

            if (UI.Instance.SettingPanel.Visible == true)
            {
                UI.Instance.UnitStats.Remove();
                UI.Instance.SettingPanel.Render();
                return;
            }
            else
            {
                UI.Instance.SettingPanel.Remove();
                UI.Instance.UnitStats.Render();
                return;
            }
        }

        internal override void Reset()
        {
            base.Reset();

            Text = Active ? "<<" : ">>";
        }
    }
}