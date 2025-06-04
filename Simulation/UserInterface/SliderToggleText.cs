namespace Simulation.UserInterface
{
    internal class SliderToggleText(
        string id,
        Text text,
        SliderToggle slider
    ) : UIEventComponent($"{id}_{text.Content}_slidertext")
    {
        /// <summary>
        /// Text label component associated with the toggle.
        /// </summary>
        internal Text Text { get; private set; } = text;

        /// <summary>
        /// Slider component used to represent and control the toggle state.
        /// </summary>
        internal SliderToggle Slider { get; private set; } = slider;

        internal bool Active
        {
            get
            {
                return Slider.Active;
            }
            set
            {
                Slider.Active = value;
            }
        }

        internal override bool LeftClick(float X, float Y)
        {
            return Slider.LeftClick(X, Y);
        }

        internal override void OnLeftClick()
        {
            base.OnLeftClick();
        }

        internal override void Hover(float X, float Y)
        {
            Slider.Hover(X, Y);
        }

        internal override void OnHover()
        {
            base.OnHover();
        }

        internal override bool LeftRelease()
        {
            return Slider.LeftRelease();
        }

        internal override void OnLeftRelease()
        {
            base.OnLeftRelease();
        }

        internal override void Interact()
        {
            Text.Interact();
            Slider.Interact();
        }

        internal override void UpdateViewport(float scale)
        {
            Text.UpdateViewport(scale);
            Slider.UpdateViewport(scale);
        }

        internal override void Render()
        {
            Text.Render();
            Slider.Render();
        }

        internal override void Reset()
        {
            Slider.Reset();
        }

        internal override void Remove()
        {
            Text.Remove();
            Slider.Remove();
        }
    }
}
