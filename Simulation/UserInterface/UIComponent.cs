namespace Simulation.UserInterface
{
    internal abstract class UIComponent(string id)
    {
        internal string ID { get; set; } = id;

        internal virtual void UpdateViewport(float scale) { }

        internal virtual void Render() { }

        internal virtual void Reset() { }

        internal virtual void Remove() { }
    }
}
