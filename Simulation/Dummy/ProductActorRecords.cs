using Akka.Actor;
using Mover = Simulation.UnitTransport.Mover;
using Producer = Simulation.UnitProduction.Producer;

namespace Simulation.Dummy
{
    /// <summary>
    /// Sent to the product actor to start its processing cycle.
    /// </summary>
    internal sealed record StartProcessing { }

    /// <summary>
    /// ProductActor → TransportManager: “Please allocate this product to the mover.”
    /// </summary>
    internal sealed record RequestTransportAllocation(IActorRef Product, Mover Mover);

    /// <summary>
    /// TransportManager → ProductActor: “Tell's the product if the mover was allocated.”
    /// </summary>
    internal sealed record TransportAllocated(bool Allocated);

    /// <summary>
    /// ProductActor → ProductionManager: “Please queue this product to the producer.”
    /// </summary>
    internal sealed record RequestQueueProduction(string ID, Producer Producer);

    /// <summary>
    /// ProductionManager → ProductActor: “Tell's the product if was queued.”
    /// </summary>
    internal sealed record ProductionQueued(bool Queued);

    /// <summary>
    /// Sent from a mover to the product actor when it has reached its destination.
    /// </summary>
    internal sealed record TransportCompleted(ulong Ticks, float Distance);

    /// <summary>
    /// Sent from a producer to the product actor when it has become blocked through user interaction.
    /// </summary>
    internal sealed record ProductionBailed();

    /// <summary>
    /// Sent from a producer to the product actor when processing of the current step is complete.
    /// </summary>
    internal sealed record ProcessingCompleted(ulong Ticks);

    /// <summary>
    /// Ask the product to shut down immediately.
    /// </summary>
    internal sealed record KillProduct;
}
