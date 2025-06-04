namespace Simulation.Unit
{
    /// <summary>
    /// Represents the possible states of a unit in the simulation.
    /// <list type="bullet">
    ///   <item><description><b>Alive:</b> Indicates that the unit is active and functioning normally.</description></item>
    ///   <item><description><b>Blocked:</b> Indicates that the unit is blocked and cannot perform its normal operations.</description></item>
    /// </list>
    /// </summary>
    internal enum State
    {
        Alive,
        Blocked
    }
}
