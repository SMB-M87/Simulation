using System.Runtime.InteropServices;

namespace Win32.Struct
{
    /// <summary>
    /// Defines a rectangle using the coordinates of its edges.
    /// <list type="bullet">
    ///   <item><description><b>Usage:</b> Commonly used in Win32 API functions to describe areas and window sizes.</description></item>
    ///   <item><description><b>Coordinates:</b> Measured in pixels relative to the client or screen space.</description></item>
    /// </list>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT
    {
        /// <summary>
        /// X-coordinate of the left edge of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Horizontal start:</b> Defines the minimum X-bound of the rectangle.</description></item>
        /// </list>
        /// </summary>
        public int left;

        /// <summary>
        /// Y-coordinate of the top edge of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Vertical start:</b> Defines the minimum Y-bound of the rectangle.</description></item>
        /// </list>
        /// </summary>
        public int top;

        /// <summary>
        /// X-coordinate of the right edge of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Horizontal end:</b> Defines the maximum X-bound of the rectangle.</description></item>
        /// </list>
        /// </summary>
        public int right;

        /// <summary>
        /// Y-coordinate of the bottom edge of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Vertical end:</b> Defines the maximum Y-bound of the rectangle.</description></item>
        /// </list>
        /// </summary>
        public int bottom;

        /// <summary>
        /// Calculates the width of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Computed:</b> Difference between <c>right</c> and <c>left</c>.</description></item>
        /// </list>
        /// </summary>
        public readonly int Width => right - left;

        /// <summary>
        /// Calculates the height of the rectangle.
        /// <list type="bullet">
        ///   <item><description><b>Computed:</b> Difference between <c>bottom</c> and <c>top</c>.</description></item>
        /// </list>
        /// </summary>
        public readonly int Height => bottom - top;
    }
}
