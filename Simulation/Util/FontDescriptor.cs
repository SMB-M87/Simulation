namespace Simulation.Util
{
    /// <summary>
    /// Describes what font and size to use when laying out text.
    /// </summary>
    internal readonly struct FontDescriptor
    {
        /// <summary>Font family name, e.g. "Arial" or "Segoe UI".</summary>
        internal string Family { get; }

        /// <summary>Font size in pixels.</summary>
        internal float SizePx { get; }

        internal FontDescriptor(string family, float sizePx)
        {
            Family = family;
            SizePx = sizePx;
        }
    }
}
