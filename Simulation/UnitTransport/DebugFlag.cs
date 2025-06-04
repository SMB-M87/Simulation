namespace Simulation.UnitTransport
{
    internal enum DebugFlag : uint
    {
        None = 0b00000,
        Velocity = 0b1,
        Acceleration = 0b10,
        Radius = 0b100,
        Detection = 0b1000,
        Path = 0b10000
    }
}
