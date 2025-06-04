namespace Win32
{
    /// <summary>
    /// Contains constant values for virtual key codes and mouse messages used in native Windows input handling.
    /// <list type="bullet">
    ///   <item><description><b>Mouse events:</b> <c>LEFT_BUTTON_PRESS</c>, <c>LEFT_BUTTON_RELEASE</c>, <c>MOUSE_MOVE</c> — used for detecting click and move actions.</description></item>
    ///   <item><description><b>Keyboard state:</b> <c>KEY_PRESS</c>, <c>KEY_RELEASE</c> — general codes for key press/release events.</description></item>
    ///   <item><description><b>Key codes:</b> <c>ESCAPE</c>, <c>CONTROL</c>, <c>SPACE</c> — commonly used keys for simulation control.</description></item>
    /// </list>
    /// For details, see <see href="https://learn.microsoft.com/en-us/windows/win32/inputdev/virtual-key-codes">Virtual Key Codes</see>.
    /// </summary>
    public static class VirtualKey
    {
        public const int LEFT_BUTTON_PRESS = 0x0201;
        public const int LEFT_BUTTON_RELEASE = 0x0202;
        public const int RIGHT_BUTTON_PRESS = 0x0204;
        public const int RIGHT_BUTTON_RELEASE = 0x0205;
        public const int MOUSE_MOVE = 0x0200;

        public const uint KEY_PRESS = 0x0100;
        public const uint KEY_RELEASE = 0x0101;

        public const int ESCAPE = 0x1B;
        public const int CONTROL = 0x11;
        public const int SPACE = 0x20;
    }
}
