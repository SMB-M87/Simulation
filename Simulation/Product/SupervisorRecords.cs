namespace Simulation.Product
{
    /// <summary>
    /// Message used to request the addition of a product with a specific type and identifier.
    /// </summary>
    /// <param name="ProductType">The type of product to be added.</param>
    internal sealed record class CreateProduct(Product ProductType) { }

    /// <summary>
    /// <list type="bullet">
    /// <item><description>Message indicating that a product has completed processing.</description></item>
    /// <item><description>Includes individual interaction metrics: ID, active ticks, travel distance, and processing ticks.</description></item>
    /// </list>
    /// </summary>
    /// <param name="Id"><description>Unique identifier for the completed product instance.</description></param>
    /// <param name="Ticks"><description>Number of ticks the product was active or processed.</description></param>
    /// <param name="Distance"><description>Total distance covered during product processing or movement.</description></param>
    /// <param name="ProcessingTicks"><description>Number of ticks spent in processing the product.</description></param>
    /// <param name="Interactions"><description>Number of interactions to process the product.</description></param>
    internal sealed record class ProductCompleted(string Id, ulong Ticks, float Distance, ulong ProcessingTicks, string Interactions) { }

    /// <summary>
    /// <list type="bullet">
    /// <item><description>Requests the supervisor to send the current completed product tracker contents.</description></item>
    /// </list>
    /// </summary>
    internal sealed record GetCompletedProductTracker();

    /// <summary>
    /// <list type="bullet">
    /// <item><description>Response containing a read-only snapshot of the completed product tracker.</description></item>
    /// </list>
    /// </summary>
    /// <param name="Tracker"><description>Dictionary mapping product IDs to their completed ticks and distance metrics.</description></param>
    internal sealed record CompletedProductTrackerSnapshot(IReadOnlyDictionary<string, (ulong Ticks, float Distance, string Interactions)> Tracker);

    /// <summary>
    /// <list type="bullet">
    /// <item><description>Message indicating that a product is currently in progress.</description></item>
    /// <item><description>Includes ongoing interaction metrics: ID, active ticks, travel distance, and processing ticks.</description></item>
    /// </list>
    /// </summary>
    /// <param name="Id"><description>Unique identifier for the product in progress.</description></param>
    /// <param name="Ticks"><description>Number of ticks the product has been active.</description></param>
    /// <param name="Distance"><description>Total distance covered so far by the product.</description></param>
    /// <param name="ProcessingTicks"><description>Number of ticks spent processing the product to date.</description></param>
    /// <param name="Interactions"><description>Number of completed interactions.</description></param>
    internal sealed record class ProductInProgress(string Id, ulong Ticks, float Distance, ulong ProcessingTicks, string Interactions) { }

    /// <summary>
    /// <list type="bullet">
    /// <item><description><b>GetInProgressProductTracker</b>: Requests the supervisor to send the current in-progress product tracker contents.</description></item>
    /// </list>
    /// </summary>
    internal sealed record GetInProgressProductTracker();

    /// <summary>
    /// <list type="bullet">
    /// <item><description><b>InProgressProductTrackerSnapshot</b>: Response containing a read-only snapshot of the in-progress product tracker.</description></item>
    /// </list>
    /// </summary>
    /// <param name="Tracker"><description>Dictionary mapping product IDs to their in-progress ticks and distance metrics.</description></param>
    internal sealed record InProgressProductTrackerSnapshot(IReadOnlyDictionary<string, (ulong Ticks, float Distance, string Interactions)> Tracker);

    internal sealed record ResetChildren();
    internal sealed record ResetSupervisorState();
}
