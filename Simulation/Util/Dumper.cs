using Akka.Actor;
using Akka.Event;
using CompletedProductTrackerSnapshot = Simulation.Product.CompletedProductTrackerSnapshot;
using GetCompletedProductTracker = Simulation.Product.GetCompletedProductTracker;
using GetInProgressProductTracker = Simulation.Product.GetInProgressProductTracker;
using InProgressProductTrackerSnapshot = Simulation.Product.InProgressProductTrackerSnapshot;

namespace Simulation.Util
{
    /// <summary>
    /// Static utility for exporting a complete snapshot of the simulation state to disk in a structured log file.
    /// Supports detailed metrics for analysis, benchmarking, and robust fallback mechanisms in case of write failures.
    /// <list type="bullet">
    ///   <item><description><b>Runtime metrics:</b> Logs tick count, frame duration, update rate, and total collisions.</description></item>
    ///   <item><description><b>Mover diagnostics:</b> Includes distance traveled, transport success rate, idle time, and efficiency statistics.</description></item>
    ///   <item><description><b>Producer diagnostics:</b> Captures processed vs. queued task counts and interaction efficiency data.</description></item>
    ///   <item><description><b>Crash-tolerant logging:</b> Automatically retries write attempts and falls back to minimal output if a full dump fails.</description></item>
    /// </list>
    /// </summary>
    internal static class Dumper
    {
        /// <summary>
        /// Number of update ticks that have occurred until the simulation run halted.
        /// <list type="bullet">
        ///   <item><description><b>Used for normalization:</b> Helps compute percentages and averages in logs.</description></item>
        ///   <item><description><b>Tracks simulation duration:</b> Represents total discrete updates since simulation start.</description></item>
        /// </list>
        /// </summary>
        private static ulong _ticks = 0;

        /// <summary>
        /// Aggregate whole‐millimeter distance traveled by all movers.
        /// <list type="bullet">
        ///   <item><description><b>Tracks global motion:</b> Total movement accumulated across all movers.</description></item>
        ///   <item><description><b>Used in performance summaries:</b> Converted to meters for reporting and benchmarking.</description></item>
        /// </list>
        /// </summary>
        private static ulong _totalTraveledMillimeters = 0;

        /// <summary>
        /// Accumulated fractional millimeter remainder, carried between movers to maintain precision.
        /// <list type="bullet">
        ///   <item><description><b>Preserves accuracy:</b> Ensures sub-millimeter movement is not lost due to rounding.</description></item>
        ///   <item><description><b>Rolled into totals:</b> Converted to whole millimeters when sufficient to increment distance.</description></item>
        /// </list>
        /// </summary>
        private static double _fractionalMillimeters = 0.0f;

        /// <summary>
        /// Aggregate count of all processing operations completed by all producers.
        /// <list type="bullet">
        ///   <item><description><b>Summed across producers:</b> Total number of interactions executed across the entire simulation.</description></item>
        ///   <item><description><b>Reported in summary:</b> Displayed in the final log output under global metrics.</description></item>
        /// </list>
        /// </summary>
        private static ulong _totalProcessedCount = 0;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_totalAssembledProducts</b>: Tracks the total number of products successfully assembled during the simulation.</description></item>
        /// </list>
        /// </summary>
        private static ulong _totalAssembledProducts = 0;

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_totalProductsInProgress</b>: Tracks the total number of products still in progress while quitting the simulation.</description></item>
        /// </list>
        /// </summary>
        private static ulong _totalProductsInProgress = 0;

        /// <summary>
        /// Formatted log lines for each mover, producer and products assembled in <see cref="GetMoverLogs"/>, <see cref="GetProducerLogs"/> and <see cref="GetAssembledProductLogs"/>.
        /// <list type="bullet">
        ///   <item><description><b>One entry per entity:</b> Captures movement and behavior metrics such as distance, idle rate, and transport history.</description></item>
        ///   <item><description><b>Grouped by model:</b> Outputs statistics grouped and labeled by mover/producer/product model for clarity.</description></item>
        ///   <item><description><b>Tabular formatting:</b> Each log line matches the format specified in <c>_moverFormat</c>, <c>_producerFormat...</c>.</description></item>
        /// </list>
        /// </summary>
        private static readonly List<string> _logs = [];

        /// <summary>
        /// Format string for mover diagnostics columns:
        /// <list type="bullet">
        ///   <item><description><b>ID:</b> Unique identifier of the mover.</description></item>
        ///   <item><description><b>Distance (m):</b> Total distance traveled, expressed in meters with fractional precision.</description></item>
        ///   <item><description><b>mm's/tick:</b> Average millimeters moved per active simulation tick.</description></item>
        ///   <item><description><b>Transports:</b> Number of transport operations completed.</description></item>
        ///   <item><description><b>Tick's/Transport:</b> Average number of ticks required per transport operation.</description></item>
        ///   <item><description><b>% Allocated:</b> Percentage of time the mover was assigned a product (active duty).</description></item>
        ///   <item><description><b>% Motionless:</b> Portion of active time spent motionless.</description></item>
        ///   <item><description><b>% Idle:</b> Percentage of time the mover was unassigned and inactive.</description></item>
        ///   <item><description><b>% Idle Moving:</b> Portion of idle time spent in motion (not parked).</description></item>
        /// </list>
        /// </summary>
        private const string _moverFormat = "{0,-15}  {1,25}  {2,13}  {3,13}  {4,18}  {5,13} %  {6,16} %  {7,10} %  {8,16} %";

        /// <summary>
        /// Format string for producer diagnostics columns:
        /// <list type="bullet">
        ///   <item><description><b>ID:</b> Unique identifier of the producer.</description></item>
        ///   <item><description><b>Interactions:</b> Total number of executed interaction operations.</description></item>
        ///   <item><description><b>Processing (ticks):</b> Cumulative number of simulation ticks spent processing.</description></item>
        ///   <item><description><b>% Processing:</b> Percentage of total ticks spent in processing mode.</description></item>
        ///   <item><description><b>% Queued:</b> Percentage of total ticks where the producer had items in queue.</description></item>
        ///   <item><description><b>% Queued Processing:</b> Percentage of queued time actively spent processing.</description></item>
        ///   <item><description><b>Interactions Overview:</b> Summary of all interaction types and their stats (count, tick cost, share).</description></item>
        /// </list>
        /// </summary>
        private const string _producerFormat = "{0,-15}  {1,15}  {2,20}  {3,15} %  {4,10} %  {5,20} %     {6,-120}";

        /// <summary>
        /// Format string for product diagnostics columns:
        /// <list type="bullet">
        ///   <item><description><b>ID:</b> Unique identifier of the product.</description></item>
        ///   <item><description><b>Active:</b> Cumulative number of tick's spent alive.</description></item>
        ///   <item><description><b>Distance:</b> Cumulative traveled distance in millimeters.</description></item>
        /// </list>
        /// </summary>
        private const string _productFormat = "{0,-30}  {1,15}  {2,20}  {3,15}";

        /// <summary>
        /// Produces a comprehensive simulation state dump and writes it to the output directory.
        /// Includes high-level metadata, runtime statistics, per-mover and per-producer summaries. 
        /// Logging is fault-tolerant with layered fallbacks.
        /// 
        /// The method proceeds as follows:
        /// <list type="number">
        ///   <item><description><b>Step 1:</b> Collects simulation data including environment IDs, tick counts, and mover/producer logs.</description></item>
        ///   <item><description><b>Step 2:</b> Attempts to write the complete log to <c>Dump.txt</c> in the simulation’s output directory.</description></item>
        ///   <item><description><b>Step 3:</b> If writing fails, tries a partial recovery to <c>Dump_crashlog_partially_recovered.txt</c>.</description></item>
        ///   <item>
        ///     <description><b>Step 4:</b> If recovery also fails, attempts minimal fallback logging to:
        ///     <list type="bullet">
        ///       <item><description><c>Dump_crashlog.txt</c> in the output directory, or</description></item>
        ///       <item><description>a timestamped crash log in the generic <c>Output</c> folder.</description></item>
        ///     </list>
        ///     </description>
        ///   </item>
        ///   <item><description><b>Step 5:</b> All major steps are written to <c>Log.txt</c> using <c>App.Log</c>.</description></item>
        /// </list>
        /// </summary>
        internal static void Log()
        {
            App.Log.Info("[Dumper] Start gathering dump log information");

            var env = Environment.Instance.ID;
            var m = Environment.Instance.Movers.Get().FirstOrDefault();
            var nav = m != null ? m.Navigation.GetID() : "";
            var mov = m != null ? m.Cost.GetID() : "";
            var p = Environment.Instance.Producers.Get().FirstOrDefault();
            var prod = p != null ? p.Cost.GetID() : "";

            try
            {
                _ticks = Cycle.UpdateTicks;
                GetProducerLogs(_ticks);
                GetMoverLogs(_ticks);
                GetAssembledProductLogs();
                GetProductInProgressLogs();

                var filePath = Path.Combine(Environment.Instance.OutputDir, "Dump.txt");
                Directory.CreateDirectory(Path.GetDirectoryName(filePath)!);

                using var writer = new StreamWriter(filePath);

                foreach (var log in GetLogs(env, nav, mov, prod))
                    writer.WriteLine(log);

                writer.Flush();
                App.Log.Info("[Dumper] Created file: {0}", filePath);
            }
            catch (Exception dumpEx)
            {
                try
                {
                    var fallback = Path.Combine(Environment.Instance.OutputDir, "Dump_crashlog_partially_recovered.txt");
                    Directory.CreateDirectory(Path.GetDirectoryName(fallback)!);

                    using var w = new StreamWriter(fallback);

                    w.WriteLine($"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] Failed to write full dump:");
                    w.WriteLine(dumpEx.ToString());
                    w.WriteLine();

                    foreach (var log in GetLogs(env, nav, mov, prod))
                        w.WriteLine(log);

                    App.Log.Info("[Dumper] Created file: {0}", fallback);
                }
                catch
                {
                    try
                    {
                        var fallback = Path.Combine(Environment.Instance.OutputDir, $"Dump_crashlog.txt");

                        Directory.CreateDirectory(Path.GetDirectoryName(fallback)!);

                        File.WriteAllText(fallback,
                            $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] Failed to write full dump:\r\n{dumpEx}");

                        App.Log.Info("[Dumper] Created file: {0}", fallback);
                    }
                    catch
                    {
                        var fallback = Path.Combine("Output", $"Dump_crashlog_{DateTime.Now:yyyyMMddHHmmss}.txt");

                        Directory.CreateDirectory(Path.GetDirectoryName(fallback)!);

                        File.WriteAllText(fallback,
                            $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] Failed to write full dump:\r\n{dumpEx}");

                        App.Log.Info("[Dumper] Created file: {0}", fallback);
                    }
                }
            }
        }

        /// <summary>
        /// Compiles a complete set of simulation dump logs, including environment metadata, runtime statistics, 
        /// and detailed per-agent and per-producer reports.
        /// 
        /// The log is organized into the following sections:
        /// <list type="bullet">
        ///   <item><description><b>Environment summary:</b> Displays simulation configuration IDs such as navigation and cost models.</description></item>
        ///   <item><description><b>Runtime stats:</b> Shows timing data including start time, stop time, paused duration, tick rate, and total collisions.</description></item>
        ///   <item><description><b>Travel stats:</b> Summarizes the total distance traveled and the number of processed interactions across all agents.</description></item>
        ///   <item><description><b>Mover table:</b> Lists metrics for each mover, such as traveled distance, transport efficiency, and idle behavior.</description></item>
        ///   <item><description><b>Producer table:</b> Lists metrics for each producer, including interaction counts, processing utilization, and detailed breakdowns.</description></item>
        /// </list>
        /// </summary>
        /// <param name="env">The simulation environment identifier, used to categorize the environment configuration.</param>
        /// <param name="nav">The navigation model used in the simulation, to indicate how agents are navigated.</param>
        /// <param name="mov">The transport cost model used by movers for simulations involving transportation.</param>
        /// <param name="prod">The production cost model used by producers, to track the operational costs of production.</param>
        /// <returns>A list of formatted strings containing the compiled logs for the entire simulation.</returns>
        private static List<string> GetLogs(
            string env,
            string nav,
            string mov,
            string prod)
        {
            var start = Cycle.StartedAt;
            var stopped = Cycle.StoppedAt;
            var paused = Cycle.PausedDuration;
            var runtime = (stopped - start) - paused;
            var tickPerSecond = _ticks / runtime.TotalSeconds;

            var totalTraveledMeters = _totalTraveledMillimeters / 1000UL;
            var leftoverMillimetersInt = _totalTraveledMillimeters % 1000UL;
            var frac = "";
            if (_fractionalMillimeters > 0.0f)
            {
                var fracStr = _fractionalMillimeters.ToString(".##");

                if (fracStr.Length > 1 && fracStr[0] == '.')
                    frac = fracStr[1..];
            }

            var logs = new List<string>
            {
                $"Environment: {env}",
                $"Navigation: {nav}",
                $"TransportCost: {mov}",
                $"ProductionCost: {prod}\n",
                $"Start: {start:yyyy-MM-dd HH:mm:ss}",
                $"End: {stopped:yyyy-MM-dd HH:mm:ss}\n",
                $"Paused: {paused:g}",
                $"Runtime: {runtime:g}\n",
                $"Update Tick's: {_ticks}",
                $"Tick's/Second: {tickPerSecond:0.##}\n",
                $"Number of Collision's: {Environment.Instance.Collisions}",
                $"Traveled Distance: {totalTraveledMeters},{leftoverMillimetersInt}{frac} meters",
                $"Processed Interactions: {_totalProcessedCount}",
                $"Assembled Products: {_totalAssembledProducts}",
                $"Products In Progress: {_totalProductsInProgress}"
            };
            foreach (var log in _logs)
                logs.Add(log);

            return logs;
        }

        /// <summary>
        /// Collects and formats logging statistics for all producers in the simulation.
        /// <list type="bullet">
        ///   <item><description><b>Group summary:</b> Organizes producers by model type, reporting counts per group.</description></item>
        ///   <item><description><b>Interaction count:</b> Total number of executed interactions per producer.</description></item>
        ///   <item><description><b>Processing (ticks):</b> Total number of ticks spent actively processing interactions.</description></item>
        ///   <item><description><b>Processing %:</b> Time spent processing as a percentage of total ticks.</description></item>
        ///   <item><description><b>Queued %:</b> Time during which the producer had pending items in queue.</description></item>
        ///   <item><description><b>Queued processing %:</b> Time spent processing while items were queued.</description></item>
        ///   <item><description><b>Interaction breakdown:</b> For each interaction type, logs how often it occurred and its cost share.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ticks"><b>Total ticks</b> in the simulation used as baseline for percentage calculations (e.g., utilization, queuing).</param>
        private static void GetProducerLogs(ulong ticks)
        {
            var producers = Environment.Instance.Producers.Get();
            _logs.Add($"\n\n=============== {producers.Count} Producer's ===============");

            if (producers == null || producers.Count == 0)
                return;

            var first = producers.FirstOrDefault();

            if (first == null)
                return;

            var current = first.Model;
            var modelGroup = producers.Where(m => m.Model == current).ToList();

            _logs.Add($"=== {modelGroup.Count} {current} ===");
            _logs.Add(
                string.Format(
                    _producerFormat,
                    "ID",
                    "Interactions",
                    "Processing (ticks)",
                    "Processing",
                    "Queued",
                    "Queued Processing",
                    "Interactions Overview"
                    )
                );

            foreach (var producer in producers.OrderBy(p => p.Model).ThenBy(p => ticks - p.EmptyQueuedCounter))
            {
                var model = producer.Model;

                if (model != current)
                {
                    modelGroup = [.. producers.Where(m => m.Model == model)];
                    _logs.Add($"\n=== {modelGroup.Count} {model} ===");
                    _logs.Add(
                        string.Format(
                            _producerFormat,
                            "ID",
                            "Interactions",
                            "Processing (ticks)",
                            "Processing",
                            "Queued",
                            "Queued Processing",
                            "Interactions Overview"
                            )
                        );
                    current = model;
                }

                var count = producer.InteractionCounter.Values.Aggregate(0UL, (sum, pair) => sum + pair.Executed);
                var activeTicks = producer.InteractionCounter.Values.Aggregate(0UL, (sum, pair) => sum + pair.Ticks);
                var rawProcessing = ticks > 0 ? (((activeTicks * 1.0f) / ticks) * 100).ToString("0.##") : "0.00";
                var queudTicks = ticks - producer.EmptyQueuedCounter;
                var queued = ticks > 0 ? (((queudTicks * 1.0f) / ticks) * 100).ToString("0.##") : "0.00";
                var queudProcessing = queudTicks > 0 ? (((activeTicks * 1.0f) / queudTicks) * 100).ToString("0.##") : "0.00";
                _totalProcessedCount += count;

                var overviewInteractions = "";

                foreach (var interaction in producer.InteractionCounter)
                {
                    overviewInteractions +=
                        string.Format(
                            "{0,-60}",
                            string.Format(
                            "{0,-1} * {1,-1} [{2,-1} tick's | {3,-1}%]",
                            interaction.Value.Executed,
                            interaction.Key,
                            interaction.Value.Ticks,
                            activeTicks > 0 ? (((interaction.Value.Ticks * 1.0f) / activeTicks) * 100).ToString("0.##") : "0.00"
                        ));
                }

                _logs.Add(string.Format(
                    _producerFormat,
                    producer.ID,
                    count,
                    activeTicks,
                    rawProcessing,
                    queued,
                    queudProcessing,
                    overviewInteractions
                ));
            }
        }

        /// <summary>
        /// Collects and formats logging statistics for all movers in the simulation.
        /// <list type="bullet">
        ///   <item><description><b>Group summary:</b> Organizes movers by model type.</description></item>
        ///   <item><description><b>Distance:</b> Total traveled distance in millimeters, formatted as meters with optional decimal precision.</description></item>
        ///   <item><description><b>Average:</b> Mean distance moved per tick during active movement (not idle or motionless).</description></item>
        ///   <item><description><b>Allocated:</b> Percentage of total ticks where the mover was assigned to a task.</description></item>
        ///   <item><description><b>Transports:</b> Total number of completed transport operations by the mover.</description></item>
        ///   <item><description><b>Ticks/Transport:</b> Average number of active ticks required per completed transport (excluding idle).</description></item>
        ///   <item><description><b>Idle:</b> Percentage of ticks where the mover was not assigned (idle).</description></item>
        ///   <item><description><b>Idle Moving:</b> Percentage of idle time where the mover was still in motion.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ticks"><b>Total simulation ticks</b> used for normalization in percentage and average calculations.</param>
        private static void GetMoverLogs(ulong ticks)
        {
            var movers = Environment.Instance.Movers.Get();

            if (movers == null || movers.Count == 0)
                return;

            _logs.Add($"\n\n=============== {movers.Count} Mover's ===============");

            var first = movers.FirstOrDefault();

            if (first == null)
                return;

            var current = first.Model;
            var modelGroup = movers.Where(m => m.Model == current).ToList();

            _logs.Add($"=== {modelGroup.Count} {current} ===");
            _logs.Add(
                string.Format(
                    _moverFormat,
                    "ID",
                    "Distance (m)",
                    "mm's/tick",
                    "Transports",
                    "Tick's/Transport",
                    "Allocated",
                    "Motionless",
                    "Idle",
                    "Idle Moving"
                    )
                );

            foreach (var mover in movers.OrderBy(m => m.Model).ThenByDescending(m => m.Idle))
            {
                var model = mover.Model;

                if (model != current)
                {
                    modelGroup = [.. movers.Where(m => m.Model == model)];
                    _logs.Add($"\n=== {modelGroup.Count} {model} ===");
                    _logs.Add(
                        string.Format(
                            _moverFormat,
                            "ID",
                            "Distance (m)",
                            "mm's/tick",
                            "Transports",
                            "Tick's/Transport",
                            "Allocated",
                            "Motionless",
                            "Idle",
                            "Idle Moving"
                            )
                        );
                    current = model;
                }

                var distance = mover.Distance;
                var active = ticks > 0 ? ticks - mover.Idle : 0;
                var average = active > 0 ? ((distance + mover.FractionalDistance) / active).ToString("0.##") : "0.00";
                var allocated = ticks > 0 ? (((ticks - mover.Idle + 0.0f) / ticks) * 100).ToString("0.##") : "0.00";
                var motionless = active > 0 ? (((mover.ActiveStationary + 0.0f) / active) * 100).ToString("0.##") : "0.00";
                var idle = ticks > 0 ? (((mover.Idle + 0.0f) / ticks) * 100).ToString("0.##") : "0.00";

                var transportingTicks = ticks - mover.Idle - mover.ActiveStationary;
                var ticksPerTransport = mover.Count > 0 ?
                    ((transportingTicks + 0.0f) / mover.Count).ToString("0.##") : "0.00";

                var idleTicks = mover.Idle;
                var idleMoving = idleTicks > 0 ?
                    (((idleTicks - mover.IdleStationary + 0.0f) /
                    idleTicks) * 100).ToString("0.##") : "0.00";

                _totalTraveledMillimeters += distance;
                _fractionalMillimeters += mover.FractionalDistance;

                if (_fractionalMillimeters >= 1.0)
                {
                    var whole = (ulong)_fractionalMillimeters;
                    _totalTraveledMillimeters += whole;
                    _fractionalMillimeters -= whole;
                }

                var totalTraveledMeters = distance / 1000UL;
                var leftoverMillimetersInt = distance % 1000UL;
                var frac = "";
                if (mover.FractionalDistance > 0.0f)
                {
                    var fracStr = mover.FractionalDistance.ToString(".##");

                    if (fracStr.Length > 1 && fracStr[0] == '.')
                        frac = fracStr[1..];
                }

                _logs.Add(string.Format(
                    _moverFormat,
                    mover.ID,
                    $"{totalTraveledMeters},{leftoverMillimetersInt}{frac}",
                    average,
                    mover.Count,
                    ticksPerTransport,
                    allocated,
                    motionless,
                    idle,
                    idleMoving
                ));
            }
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>GetAssembledProductLogs</b>: Retrieves and logs metrics for all completed products.</description></item>
        /// <item><description>Sends a <see cref="GetCompletedProductTracker"/> request and awaits a <see cref="CompletedProductTrackerSnapshot"/> response.</description></item>
        /// <item><description>Validates response data and skips logging if empty.</description></item>
        /// <item><description>Extracts product type prefixes from IDs and groups entries by type, ordering by tick count.</description></item>
        /// <item><description>Logs total count, then formats and appends each product’s ID, active ticks, and distance to the log.</description></item>
        /// </list>
        /// </summary>
        private static void GetAssembledProductLogs()
        {
            var response = App.ProductSupervisor
                .Ask<CompletedProductTrackerSnapshot>(new GetCompletedProductTracker(), Timeout.InfiniteTimeSpan).Result;

            var products = response?.Tracker;
            if (products == null || products.Count == 0)
                return;

            var firstKey = products.Keys.FirstOrDefault();
            if (string.IsNullOrWhiteSpace(firstKey))
                return;

            static string GetPrefix(string key)
            {
                var idx = key.IndexOf('_');
                return idx >= 0
                    ? key[..idx]
                    : key;
            }

            _logs.Add($"\n\n=============== {products.Count} Assembled Product's ===============");
            _totalAssembledProducts = (ulong)products.Count;

            var grouped = products
                .OrderBy(p => GetPrefix(p.Key))
                .ThenBy(p => p.Value.Ticks)
                .GroupBy(p => GetPrefix(p.Key))
                .ToList();

            foreach (var group in grouped)
            {
                _logs.Add($"[{group.Count()} {group.Key}]");
                var totalDistance = group.Sum(kv => kv.Value.Distance);
                var totalTicks = group.Sum(kv => (decimal)kv.Value.Ticks);
                var avgDistance = totalDistance / group.Count();
                var avgTicks = totalTicks / group.Count();
                var fastestTicks = group.Min(kv => kv.Value.Ticks);
                var slowestTicks = group.Max(kv => kv.Value.Ticks);
                var shortestDistance = group.Min(kv => kv.Value.Distance);
                var longestDistance = group.Max(kv => kv.Value.Distance);
                var bestCombinedEntry = group
                    .OrderBy(kv => kv.Value.Ticks + kv.Value.Distance)
                    .First();
                var worstCombinedEntry = group
                    .OrderByDescending(kv => kv.Value.Ticks + kv.Value.Distance)
                    .First();

                _logs.Add($"Total   : Distance = {totalDistance:0.##} mm, Ticks = {totalTicks:0.##}");
                _logs.Add($"Average : Distance = {avgDistance:0.##} mm, Ticks = {avgTicks:0.##}");
                _logs.Add($"Fastest : {fastestTicks:0.##} ticks   | Slowest   : {slowestTicks:0.##} ticks");
                _logs.Add($"Shortest: {shortestDistance:0.##} mm      | Longest   : {longestDistance:0.##} mm");
                _logs.Add($"Best Combined  : {bestCombinedEntry.Key} → " +
                          $"{bestCombinedEntry.Value.Ticks:0.##} ticks + " +
                          $"{bestCombinedEntry.Value.Distance:0.##} mm");
                _logs.Add($"Worst Combined : {worstCombinedEntry.Key} → " +
                          $"{worstCombinedEntry.Value.Ticks:0.##} ticks + " +
                          $"{worstCombinedEntry.Value.Distance:0.##} mm");
                _logs.Add("");
            }

            foreach (var group in grouped)
            {
                _logs.Add($"[{group.Count()} {group.Key}]");

                _logs.Add(string.Format(
                    _productFormat,
                    "ID",
                    "Active (ticks)",
                    "Distance (mm)",
                    "Interactions"
                ));

                foreach (var kv in group.OrderBy(kv => kv.Value.Ticks + kv.Value.Distance))
                {
                    _logs.Add(string.Format(
                        _productFormat,
                        kv.Key,
                        kv.Value.Ticks.ToString("0.##"),
                        kv.Value.Distance.ToString("0.##"),
                        kv.Value.Interactions
                    ));
                }
                _logs.Add("");
            }
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>GetProductInProgressLogs</b>: Retrieves and logs metrics for products currently in progress.</description></item>
        /// <item><description>Sends a <see cref="GetInProgressProductTracker"/> request and awaits an <see cref="InProgressProductTrackerSnapshot"/> response.</description></item>
        /// <item><description>Returns early if the response is null or contains no entries.</description></item>
        /// <item><description>Extracts product type prefixes from IDs for grouping.</description></item>
        /// <item><description>Groups products by type, orders by tick count, and counts per group.</description></item>
        /// <item><description>Logs a header and formatted details (ID, active ticks, distance, interactions) for each group.</description></item>
        /// </list>
        /// </summary>
        private static void GetProductInProgressLogs()
        {
            var response = App.ProductSupervisor
                .Ask<InProgressProductTrackerSnapshot>(new GetInProgressProductTracker(), Timeout.InfiniteTimeSpan).Result;

            var products = response?.Tracker;
            if (products == null || products.Count == 0)
                return;

            var firstKey = products.Keys.FirstOrDefault();
            if (string.IsNullOrWhiteSpace(firstKey))
                return;

            static string GetPrefix(string key)
            {
                var idx = key.IndexOf('_');
                return idx >= 0
                    ? key[..idx]
                    : key;
            }

            _logs.Add($"\n=============== {products.Count} Product's In Progress ===============");
            _totalProductsInProgress = (ulong)products.Count;

            var grouped = products
                .OrderBy(p => GetPrefix(p.Key))
                .ThenBy(p => p.Value.Ticks)
                .GroupBy(p => GetPrefix(p.Key))
                .ToList();

            foreach (var group in grouped)
            {
                _logs.Add($"[{group.Count()} {group.Key}]");
                _logs.Add(string.Format(
                    _productFormat,
                    "ID",
                    "Active (ticks)",
                    "Distance (mm)",
                    "Interactions"
                ));

                foreach (var kv in group)
                {
                    _logs.Add(string.Format(
                        _productFormat,
                        kv.Key,
                        kv.Value.Ticks.ToString("0.##"),
                        kv.Value.Distance.ToString("0.##"),
                        kv.Value.Interactions
                    ));
                }
                _logs.Add("");
            }
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Clear</b>: Resets all simulation metrics and clears the log history.</description></item>
        /// <item><description>Sets traveled and fractional millimeter counters to zero.</description></item>
        /// <item><description>Resets processed count, assembled products, and in-progress products to zero.</description></item>
        /// <item><description>Clears the accumulated log entries.</description></item>
        /// </list>
        /// </summary>
        internal static void Clear()
        {
            _totalTraveledMillimeters = 0;
            _fractionalMillimeters = 0.0f;
            _totalProcessedCount = 0;
            _totalAssembledProducts = 0;
            _totalProductsInProgress = 0;
            _logs.Clear();
        }
    }
}