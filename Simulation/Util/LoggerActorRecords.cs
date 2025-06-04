
namespace Simulation.Util
{
    /// <summary>
    /// Message instructing the <see cref="LoggerActor"/> to dispose and remove a specific writer 
    /// from its active log file dictionary, based on a unique key.
    /// </summary>
    /// <param name="Key">The dictionary key used to identify the log writer (typically folder + actor name).</param>
    internal sealed record RemoveWriter(string Key);

    /// <summary>
    /// Message instructing the <see cref="LoggerActor"/> to close the existing shared log writer
    /// and prepare for logging in a new directory.
    /// </summary>
    /// <param name="OldDir">The path to the current directory where the common log is being written.</param>
    /// <param name="NewDir">The path to the new directory where logging should continue.</param>
    public sealed record CloseCommonWriter(string OldDir, string NewDir) { }

    /// <summary>
    /// <list type="bullet">
    /// <item><description><b>ResetLogger</b>: Message used to instruct the logging system to clear or reinitialize its state.</description></item>
    /// </list>
    /// </summary>
    public sealed record ResetLogger() { }
}
