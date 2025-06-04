using System.Collections.Concurrent;
using IDWriteFactory = Vortice.DirectWrite.IDWriteFactory;
using IDWriteTextFormat = Vortice.DirectWrite.IDWriteTextFormat;

namespace Win32
{
    /// <summary>
    /// Maintains a thread-safe cache of <see cref="IDWriteTextFormat"/> instances,
    /// keyed by <see cref="TextStyle"/>.  Formats are created on demand via the
    /// provided <see cref="IDWriteFactory"/>, and disposed of when this cache is disposed.
    /// </summary>
    internal sealed class TextFormatCache(IDWriteFactory dw) : IDisposable
    {
        /// <summary>
        /// Underlying DirectWrite factory used to create new <see cref="IDWriteTextFormat"/> instances.
        /// </summary>
        private readonly IDWriteFactory _dw = dw;

        /// <summary>
        /// Cache of text formats, keyed by <see cref="TextStyle"/>.  
        /// Ensures each unique style is only created once and then reused.
        /// </summary>
        private readonly ConcurrentDictionary<TextStyle, IDWriteTextFormat> _formats = [];

        /// <summary>
        /// Retrieves an <see cref="IDWriteTextFormat"/> for the specified <paramref name="style"/>,
        /// creating and caching it if necessary.
        /// </summary>
        /// <param name="style">Font family, size, weight, style, and stretch describing the text format.</param>
        /// <returns>An <see cref="IDWriteTextFormat"/> matching the requested style.</returns>
        public IDWriteTextFormat Get(TextStyle style) =>
            _formats.GetOrAdd(style, st =>
                _dw.CreateTextFormat(st.FontFamily, null!,
                                     st.Weight, st.Style, st.Stretch, st.SizePx));

        /// <summary>
        /// Disposes all cached <see cref="IDWriteTextFormat"/> instances.
        /// </summary>
        public void Dispose()
        {
            foreach (var fmt in _formats.Values) fmt.Dispose();
            _formats.Clear();
        }
    }
}
