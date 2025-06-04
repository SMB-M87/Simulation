namespace Simulation.Tick
{
    /// <summary>
    /// Message sent to trigger a visual render tick in the simulation.
    /// Handled by the <see cref="CycleActor"/> responsible for updating the screen.
    /// </summary>
    internal sealed record RenderTick { }

    /// <summary>
    /// Message sent to trigger a UI counter update, keeping track of the <see cref="CycleActor"/> FPS/UPS performance metrics.
    /// </summary>
    internal sealed record CounterTick { }

    /// <summary>
    /// Message sent to trigger a simulation update (UPS cycle) handled by the <see cref="CycleActor"/>.
    /// </summary>
    internal sealed record UpdateTick { }

    /// <summary>
    /// Message used to request the current number of update ticks from the cycle actor
    /// so it can be cached or logged before shutdown.
    /// </summary>
    internal sealed record FetchUpdateTicks { }

    /// <summary>
    /// Tells the <see cref="CycleActor"/> to reset the update tick count.
    /// </summary>
    internal sealed record ResetCycle { }

    /// <summary>
    /// Tells the <see cref="CycleActor"/> to shut down all timers and stop itself.
    /// </summary>
    internal sealed record StopCycle { }
}
