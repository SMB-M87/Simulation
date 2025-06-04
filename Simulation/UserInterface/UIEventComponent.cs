namespace Simulation.UserInterface
{
    internal abstract class UIEventComponent(string id) : UIComponent(id.Replace(" ", ""))
    {
        internal virtual bool LeftClick(float X, float Y) { return false; }
        internal virtual void OnLeftClick() { }
        internal virtual bool LeftRelease() { return false; }
        internal virtual void OnLeftRelease() { }

        internal virtual bool RightClick(float X, float Y) { return false; }
        internal virtual void OnRightClick() { }
        internal virtual bool RightRelease() { return false; }
        internal virtual void OnRightRelease() { }

        internal virtual void Hover(float X, float Y) { }
        internal virtual void OnHover() { }

        internal virtual void Interact() { }
    }
}
