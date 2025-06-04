using FontStretch = Vortice.DirectWrite.FontStretch;
using FontStyle = Vortice.DirectWrite.FontStyle;
using FontWeight = Vortice.DirectWrite.FontWeight;

namespace Win32
{
    /// <summary>
    /// Immutable description of a text style, including font family, size in pixels,
    /// weight, style, and stretch.  Used as a key for <see cref="TextFormatCache"/>.
    /// </summary>
    /// <param name="FontFamily">Name of the font family (e.g., "Arial").</param>
    /// <param name="SizePx">Font size in pixels.</param>
    /// <param name="Weight">Text weight (normal, bold, etc.).</param>
    /// <param name="Style">Text style (normal, italic, etc.).</param>
    /// <param name="Stretch">Font stretch (normal, condensed, expanded, etc.).</param>
    public readonly record struct TextStyle(
    string FontFamily,
    float SizePx,
    FontWeight Weight = FontWeight.Normal,
    FontStyle Style = FontStyle.Normal,
    FontStretch Stretch = FontStretch.Normal)
    {
        /// <summary>
        /// Returns a new <see cref="TextStyle"/> with its <see cref="SizePx"/> scaled
        /// by the given <paramref name="factor"/>.
        /// </summary>
        /// <param name="factor">Multiplicative scale factor for the font size.</param>
        public TextStyle Scale(float factor) =>
            this with { SizePx = SizePx * factor };
    }
}
