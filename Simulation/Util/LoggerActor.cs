using Akka.Actor;
using Akka.Event;
using InitializeLogger = Akka.Event.InitializeLogger;
using LogEvent = Akka.Event.LogEvent;
using LoggerInitialized = Akka.Event.LoggerInitialized;

namespace Simulation.Util
{
    /// <summary>
    /// Custom Akka.NET logger actor responsible for structured simulation logging.
    /// Organizes log output into per-actor files and central diagnostics,
    /// and ensures critical messages are redundantly recorded for reliability.
    /// 
    /// This logger supports:
    /// <list type="bullet">
    ///   <item><description><b>Per-actor logs:</b> Writes messages from domain-specific actors (e.g., "Products") into dedicated log files.</description></item>
    ///   <item><description><b>Central log file:</b> Captures all messages from the root actor and those with severity <c>Warning</c> or higher.</description></item>
    ///   <item><description><b>Dynamic routing:</b> Filters and routes logs based on actor origin and message severity.</description></item>
    ///   <item><description><b>File management:</b> Automatically manages writer creation, flushing, and disposal to prevent resource leaks.</description></item>
    ///   <item><description><b>Akka integration:</b> Subscribes to <see cref="LogEvent"/>, <see cref="RemoveWriter"/>, and <see cref="CloseCommonWriter"/> through the event stream.</description></item>
    /// </list>
    /// </summary>
    public sealed class LoggerActor : ReceiveActor, ILogReceive
    {
        /// <summary>
        /// A dictionary that maps unique log keys (e.g., "Products|ActorName") to their associated <see cref="TextWriter"/> instances.
        /// <list type="bullet">
        ///   <item><description><b>Key format:</b> Combines log category and actor name to ensure uniqueness (e.g., "Movers|Mover_5").</description></item>
        ///   <item><description><b>Purpose:</b> Caches active <see cref="TextWriter"/> objects to prevent duplicate file handles.</description></item>
        ///   <item><description><b>Scope:</b> Used internally by logging utilities to write per-actor or per-category log output.</description></item>
        /// </list>
        /// </summary>
        private readonly Dictionary<string, TextWriter> _writers = [];

        /// <summary>
        /// Shared <see cref="TextWriter"/> used to log critical or global messages:
        /// <list type="bullet">
        ///   <item><description>Messages from the "root" (system) actor.</description></item>
        ///   <item><description>Messages from any actor with <c>LogLevel.Warning</c> or higher severity.</description></item>
        /// </list>
        /// Ensures important events are always captured in a central log, even if no per-actor log is defined.
        /// </summary>
        private TextWriter? _commonWriter;

        /// <summary>
        /// Flag indicating whether the logger is currently performing a file move operation.
        /// <list type="bullet">
        ///   <item><description><b>True:</b> Suppresses all incoming log writes during file relocation to prevent corruption.</description></item>
        ///   <item><description><b>False:</b> Normal logging behavior resumes; all messages are processed and written.</description></item>
        /// </list>
        /// </summary>
        private bool _moving = false;

        /// <summary>
        /// Character array used to split actor paths in log source strings.
        /// <list type="bullet">
        ///   <item><description><b>Purpose:</b> Allows parsing of Akka.NET actor names from full hierarchical paths.</description></item>
        ///   <item><description><b>Delimiter:</b> The forward slash <c>'/'</c> is the default separator in actor paths.</description></item>
        /// </list>
        /// </summary>
        private static readonly char[] _separator = ['/'];

        /// <summary>
        /// Initializes the <see cref="LoggerActor"/> and sets up handlers for logging-related Akka.NET messages:
        /// <list type="bullet">
        ///   <item><description><b><see cref="InitializeLogger"/>:</b> Responds with <c>LoggerInitialized</c> to confirm readiness.</description></item>
        ///   <item>
        ///     <description><b><see cref="LogEvent"/>:</b> Processes incoming log messages:
        ///       <list type="bullet">
        ///         <item><description>Writes all system/root logs to <c>Log.txt</c>.</description></item>
        ///         <item><description>Writes logs of <c>Warning</c> severity or higher to <c>Log.txt</c>.</description></item>
        ///         <item><description>Routes logs from "Products" actors to separate per-actor files.</description></item>
        ///       </list>
        ///     </description>
        ///   </item>
        ///   <item><description><b><see cref="RemoveWriter"/>:</b> Disposes and removes a per-actor log writer from the cache.</description></item>
        ///   <item><description><b><see cref="CloseCommonWriter"/>:</b> Disposes the shared log writer and moves the log file to a new directory.</description></item>
        /// </list>
        /// </summary>
        public LoggerActor()
        {
            Receive<InitializeLogger>(_ =>
            {
                Sender.Tell(new LoggerInitialized());
            });

            Receive<LogEvent>(logEvent =>
            {
                if (_moving)
                    return;

                var sourceStr = logEvent.LogSource.ToString()!;
                var messageStr = logEvent.Message.ToString()!;

                var parts = sourceStr.Split(_separator, StringSplitOptions.RemoveEmptyEntries);
                var actorName = parts.Last();
                var isSystem = actorName.Contains("root");

                var level = logEvent.LogLevel();
                var loggerId = isSystem ? "i4sim" : actorName;
                var header = $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss.ffff}][{loggerId}][{level}] ";

                if (isSystem || level >= LogLevel.WarningLevel)
                {
                    EnsureCommonWriter();

                    foreach (var line in messageStr.Split('\n', StringSplitOptions.RemoveEmptyEntries))
                        _commonWriter!.WriteLine(header + line.TrimEnd('\r'));
                }

                if (parts.Length >= 4 && (parts[3] == "Products" || parts[3] == "MQTT"))
                {
                    var key = $"{parts[3]}|{actorName}";
                    var writer = GetOrCreateWriter(key, parts[3], actorName);

                    foreach (var line in messageStr.Split('\n', StringSplitOptions.RemoveEmptyEntries))
                        writer.WriteLine(header + line.TrimEnd('\r'));
                }
            });

            Receive<RemoveWriter>(msg =>
            {
                if (_writers.TryGetValue(msg.Key, out var writer))
                {
                    writer.Dispose();
                    _writers.Remove(msg.Key);
                }
            });

            Receive<CloseCommonWriter>(msg =>
            {
                _moving = true;
                _commonWriter?.Dispose();
                _commonWriter = null;

                var oldLog = Path.Combine(msg.OldDir, "Log.txt");

                if (File.Exists(oldLog))
                {
                    Directory.CreateDirectory(msg.NewDir);

                    var newLog = Path.Combine(msg.NewDir, "Log.txt");
                    File.Move(oldLog, newLog);

                    Directory.Delete(msg.OldDir, recursive: true);
                }

                _moving = false;
            });

            Receive<ResetLogger>(_ =>
            {
                _commonWriter?.Dispose();
                _commonWriter = null;

                foreach (var writer in _writers.Values)
                    writer.Dispose();

                _writers.Clear();
            });
        }

        /// <summary>
        /// Subscribes the logger actor to the Akka.NET event stream for global logging and cleanup messages:
        /// <list type="bullet">
        ///   <item><description><b><see cref="LogEvent"/>:</b> Enables reception of system-wide and actor-level log messages.</description></item>
        ///   <item><description><b><see cref="RemoveWriter"/>:</b> Handles disposal of actor-specific log writers upon request.</description></item>
        ///   <item><description><b><see cref="CloseCommonWriter"/>:</b> Triggers shutdown of the shared writer used for general logs.</description></item>
        /// </list>
        /// This ensures the logger is fully connected to relevant message streams immediately after actor startup.
        /// </summary>
        protected override void PreStart()
        {
            Context.System.EventStream.Subscribe(Self, typeof(LogEvent));
            Context.System.EventStream.Subscribe(Self, typeof(RemoveWriter));
            Context.System.EventStream.Subscribe(Self, typeof(CloseCommonWriter));
            Context.System.EventStream.Subscribe(Self, typeof(ResetLogger));
            base.PreStart();
        }

        /// <summary>
        /// Ensures the shared log writer (<see cref="_commonWriter"/>) is initialized for general logging.
        /// <list type="bullet">
        ///   <item><description><b>Directory:</b> Creates the simulation output folder if it doesn’t exist.</description></item>
        ///   <item><description><b>File:</b> Opens or creates <c>Log.txt</c> at the end of the file for appending logs.</description></item>
        ///   <item><description><b>Writer:</b> Uses a synchronized <see cref="StreamWriter"/> with <c>AutoFlush</c> enabled for real-time logging.</description></item>
        /// </list>
        /// </summary>
        private void EnsureCommonWriter()
        {
            if (_commonWriter == null)
            {
                var dir = Path.Combine(AppContext.BaseDirectory, Environment.Instance.OutputDir);
                Directory.CreateDirectory(dir);

                var file = Path.Combine(dir, "Log.txt");
                var fs = new FileStream(
                    file,
                    FileMode.OpenOrCreate,
                    FileAccess.Write,
                    FileShare.ReadWrite | FileShare.Delete
                );

                fs.Seek(0, SeekOrigin.End);
                _commonWriter = TextWriter.Synchronized(new StreamWriter(fs) { AutoFlush = true });
            }
        }

        /// <summary>
        /// Retrieves a cached <see cref="TextWriter"/> for the specified actor log file,
        /// or creates a new one under the simulation’s output directory if not present.
        /// <list type="bullet">
        ///   <item><description><b>Key:</b> Uniquely identifies the writer by combining folder name and actor name.</description></item>
        ///   <item><description><b>Path:</b> Log file is created in a structured subdirectory under the simulation's output folder.</description></item>
        ///   <item><description><b>Writer:</b> Configured with <c>AutoFlush</c> and file-sharing settings to support live writing and reading.</description></item>
        /// </list>
        /// </summary>
        /// <param name="key">Unique dictionary key for writer lookup (typically folder + actor name).</param>
        /// <param name="folder">Optional folder to organize logs (e.g., "Products").</param>
        /// <param name="actorName">The name of the actor whose logs will be written.</param>
        /// <returns>A thread-safe <see cref="TextWriter"/> ready for logging.</returns>
        private TextWriter GetOrCreateWriter(string key, string folder, string actorName)
        {
            if (!_writers.TryGetValue(key, out var writer))
            {
                if (folder == actorName || folder == "MQTT")
                {
                    EnsureCommonWriter();
                    return _commonWriter!;
                }

                var dir = Path.Combine(AppContext.BaseDirectory, Environment.Instance.OutputDir, folder);
                Directory.CreateDirectory(dir);

                var file = Path.Combine(dir, $"{actorName}.txt");
                var fs = new FileStream(
                    file,
                    FileMode.OpenOrCreate,
                    FileAccess.Write,
                    FileShare.ReadWrite | FileShare.Delete
                );

                fs.Seek(0, SeekOrigin.End);
                writer = TextWriter.Synchronized(new StreamWriter(fs) { AutoFlush = true });
                _writers[key] = writer;
            }
            return writer;
        }

        /// <summary>
        /// Cleans up the logger actor’s resources when it stops.
        /// <list type="bullet">
        ///   <item><description><b>Writers:</b> Disposes all open <see cref="TextWriter"/> instances to release file handles.</description></item>
        ///   <item><description><b>Cleanup:</b> Clears the internal writer cache.</description></item>
        ///   <item><description><b>Safety:</b> Ignores exceptions to avoid interrupting shutdown flow.</description></item>
        /// </list>
        /// </summary>
        protected override void PostStop()
        {
            foreach (var writer in _writers.Values)
            {
                try { writer.Dispose(); }
                catch { }
            }
            _writers.Clear();
            base.PostStop();
        }
    }
}
