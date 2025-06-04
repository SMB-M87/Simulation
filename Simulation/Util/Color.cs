using Vector4 = System.Numerics.Vector4;

namespace Simulation.Util
{
    /// <summary>
    /// Provides a centralized palette of predefined <see cref="Vector4"/> values 
    /// used for rendering and visual consistency within the simulation.
    ///
    /// <list type="bullet">
    ///   <item><description><b>Readable names:</b> Color names reflect hue and opacity (e.g., <c>White75</c> is 75% opaque white).</description></item>
    ///   <item><description><b>Rendering consistency:</b> Promotes unified styling across all visual elements.</description></item>
    ///   <item><description><b>Maintainability:</b> Allows global palette changes without modifying each usage site.</description></item>
    /// </list>
    /// </summary>
    internal static class Color
    {
        public static readonly Vector4 Transparent = new(0.0f, 0.0f, 0.0f, 0.0f);

        public static readonly Vector4 White = new(1.0f, 1.0f, 1.0f, 1.0f);
        public static readonly Vector4 White75 = new(1.0f, 1.0f, 1.0f, 0.75f);
        public static readonly Vector4 White45 = new(1.0f, 1.0f, 1.0f, 0.45f);

        public static readonly Vector4 Black = new(0.0f, 0.0f, 0.0f, 1.0f);
        public static readonly Vector4 Black75 = new(0.0f, 0.0f, 0.0f, 0.75f);
        public static readonly Vector4 Black50 = new(0.0f, 0.0f, 0.0f, 0.5f);

        public static readonly Vector4 Gray85 = new(0.3f, 0.3f, 0.3f, 0.85f);
        public static readonly Vector4 Gray50 = new(0.3f, 0.3f, 0.3f, 0.5f);
        public static readonly Vector4 Gray30 = new(0.3f, 0.3f, 0.3f, 0.3f);
        public static readonly Vector4 GrayDark = new(0.2f, 0.2f, 0.25f, 1.0f);
        public static readonly Vector4 GrayLight = new(0.7f, 0.7f, 0.7f, 1.0f);
        public static readonly Vector4 GrayLight15 = new(0.7f, 0.7f, 0.7f, 0.15f);

        public static readonly Vector4 Cyan = new(0.0f, 1.0f, 1.0f, 1.0f);
        public static readonly Vector4 Cyan75 = new(0.0f, 0.65f, 0.65f, 0.75f);

        public static readonly Vector4 Green = new(0.5f, 1.0f, 0.0f, 1.0f);

        public static readonly Vector4 YellowGreen = new(0.8f, 0.7f, 0.1f, 1.0f);
        public static readonly Vector4 YellowGreen45 = new(0.8f, 0.7f, 0.1f, 0.45f);

        public static readonly Vector4 Yellow50 = new(0.65f, 0.6f, 0.25f, 0.5f);

        public static readonly Vector4 Orange75 = new(1.0f, 0.5f, 0.0f, 0.75f);
        public static readonly Vector4 OrangeDark = new(0.75f, 0.375f, 0.0f, 1.0f);
        public static readonly Vector4 OrangeDark45 = new(0.75f, 0.375f, 0.0f, 0.45f);
        public static readonly Vector4 Trump50 = new(0.85f, 0.5f, 0.0f, 0.5f);

        public static readonly Vector4 Red = new(1.0f, 0.0f, 0.0f, 1.0f);
        public static readonly Vector4 Red75 = new(1.0f, 0.0f, 0.0f, 0.75f);
        public static readonly Vector4 Red50 = new(0.9f, 0.0f, 0.0f, 0.5f);

        public static readonly Vector4 BlueDark = new(0.05f, 0.15f, 0.45f, 1.0f);
        public static readonly Vector4 Blueatre = new(0.35f, 0.5f, 0.65f, 1.0f);
        public static readonly Vector4 Blueatre45 = new(0.35f, 0.5f, 0.65f, 0.45f);

        public static readonly Vector4 Purple = new(0.75f, 0.25f, 0.5f, 1.0f);
    }
}
