using Akka.Actor;
using KillProduct = Simulation.Dummy.KillProduct;
using Mover = Simulation.UnitTransport.Mover;
using Producer = Simulation.UnitProduction.Producer;
using ProductionBailed = Simulation.Dummy.ProductionBailed;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UserInterface.Components
{
    /// <summary>
    /// A UI interaction handler that allows the user to toggle the state of a producer 
    /// in the simulation by clicking on it.
    /// 
    /// When clicked, this component checks all producers in the environment and triggers
    /// a state change for the first one that was hit by the mouse coordinates.
    /// This component does not render any UI of its own and is only active when the 
    /// settings panel is not shown.
    /// </summary>
    /// <param name="id">A unique identifier for the toggle (unused in this component).</param>
    internal class UnitToggleState(string id) : UIEventComponent($"{id}_unit_toggle")
    {
        /// <summary>
        /// Reference to the producer currently awaiting release, if any.
        /// </summary>
        private Producer? _producer;

        /// <summary>
        /// Reference to the mover currently awaiting release, if any.
        /// </summary>
        private Mover? _mover;

        /// <summary>
        /// Handles a mouse click event. If the simulation is running and the settings panel is not shown, 
        /// it processes the click to toggle the state of a producer unit within the environment based on the mouse's position.
        /// </summary>
        /// <param name="X">X-coordinate of the mouse in screen space.</param>
        /// <param name="Y">Y-coordinate of the mouse in screen space.</param>
        internal override bool RightClick(float X, float Y)
        {
            if (!Cycle.IsRunning || UI.Instance.SettingButton.Active)
                return false;

            var pos = Renderer.Instance.ScreenToWorld(new Vector2(X, Y));

            foreach (var producer in Environment.Instance.Producers.GetNearbyProducers(pos))
                if (producer.PositionInsideUnit(new(X, Y)))
                {
                    _producer = producer;
                    return true;
                }

            foreach (var mover in Environment.Instance.Movers.GetNearbyMovers(pos))
                if (mover.ScreenPointInsideUnitWorldSpace(new(X, Y)))
                {
                    _mover = mover;
                    return true;
                }

            return false;
        }

        /// <summary>
        /// Processes a release action for the currently active producer or mover.
        /// Ensures the cycle is running and no settings panel is visible.
        /// <list type="bullet">
        ///   <item><description>If a producer is active: toggles its state, renders it, notifies any queued movers of a production bail,
        ///     clears the producer’s queue, and resets the producer reference.</description></item>
        ///   <item><description>If a mover is active: updates its hard/blocked state, stops movement if entering hard state,
        ///     sends a kill product message, renders it, removes it from its producer’s queue, and resets the mover reference.</description></item>
        /// </list>
        /// </summary>
        internal override bool RightRelease()
        {
            if (!Cycle.IsRunning || UI.Instance.SettingButton.Active)
                return false;

            if (_producer != null)
            {
                _producer.ToggleState();
                _producer.Render();

                foreach (var id in _producer.Queue)
                {
                    var mover = Environment.Instance.Movers.Get(id);
                    mover?.ServiceRequester.Tell(new ProductionBailed());
                    mover?.InteractionBailed();
                }
                _producer.Queue.Clear();
                _producer = null;

                return true;
            }
            else if (_mover != null)
            {
                if (!_mover.Disabled && _mover.State == Unit.State.Blocked)
                {
                    _mover.Disabled = true;
                }
                else if (!_mover.Disabled && _mover.State == Unit.State.Alive)
                {
                    _mover.State = Unit.State.Blocked;
                    _mover.Disabled = true;
                }
                else
                {
                    _mover.Destination = Vector2.Zero;
                    _mover.State = Unit.State.Alive;
                    _mover.Disabled = false;
                }

                if (_mover.Disabled)
                {
                    _mover.ServiceRequester.Tell(new KillProduct());
                    _mover.Path = [];
                }
                _mover.Render();

                if (_mover.ServiceRequester != ActorRefs.Nobody)
                {
                    var producer = Environment.Instance.Producers.Get(_mover.ServiceRequester.Path.Name);
                    producer?.Queue.Remove(_mover.ServiceRequester.Path.Name);
                }
                _mover = null;

                return true;
            }
            return false;
        }
    }
}
