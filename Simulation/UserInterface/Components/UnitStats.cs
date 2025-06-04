using System.Collections.Concurrent;
using ActorRefs = Akka.Actor.ActorRefs;
using Colors = Simulation.Util.Color;
using Mover = Simulation.UnitTransport.Mover;
using Producer = Simulation.UnitProduction.Producer;
using TextStyles = Simulation.Util.TextStyles;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UserInterface.Components
{
    /// <summary>
    /// Displays real-time debug stats when clicking/hovering over units (movers or producers) in the simulation viewport.
    /// Triggered by mouse interactions and dynamically renders a styled tooltip with relevant unit data.
    /// <list type="bullet">
    ///   <item><description><b>Unit detection:</b> Identifies whether the cursor overlaps with a mover or producer based on render bounds.</description></item>
    ///   <item><description><b>Stat rendering:</b> Draws a rounded rectangle tooltip containing dynamic properties such as position, velocity, queue state, etc.</description></item>
    ///   <item><description><b>Per-frame update:</b> Updates hover information every frame based on cursor movement and simulation zoom state.</description></item>
    ///   <item><description><b>Display clearing:</b> Automatically removes previous draw commands when the hovered unit changes or no unit is detected.</description></item>
    /// </list>
    /// </summary>
    internal class UnitStats(string id) : UIEventComponent($"{id}_stats")
    {
        private readonly Vector2 _rectRadius = new(8, 8);

        internal int _currentCount = 0;
        private readonly object _lock = new();
        private readonly ConcurrentDictionary<string, object> _selectedUnits = new();

        internal bool Active { get => !_selectedUnits.IsEmpty; }

        /// <summary>
        /// Handles click detection for movers and producers and displays a stats panel near the cursor.
        /// If nothing is clicked, it doesn't clears the previous stat panel.
        /// <list type="bullet">
        ///   <item><description><b>UI toggle check:</b> If the settings UI is currently shown, no click stats are displayed.</description></item>
        ///   <item><description><b>Mover detection:</b> Checks if the cursor is over any mover; if so, draws its stats panel.</description></item>
        ///   <item><description><b>Producer detection:</b> If no mover matched, checks for a producer under the cursor and draws its stats.</description></item>
        ///   <item><description><b>Cleanup:</b> If no unit is under the cursor, removes the previously drawn hover stats panel.</description></item>
        /// </list>
        /// </summary>
        /// <param name="mouseX">The X coordinate of the mouse in screen space.</param>
        /// <param name="mouseY">The Y coordinate of the mouse in screen space.</param>
        internal override bool LeftClick(float mouseX, float mouseY)
        {
            if (!Cycle.IsRunning || UI.Instance.SettingButton.Active)
                return false;

            object? unit = null;
            _currentCount = _selectedUnits.Count;
            var pos = Renderer.Instance.ScreenToWorld(new Vector2(mouseX, mouseY));

            foreach (var mover in Environment.Instance.Movers.GetNearbyMovers(pos))
            {
                if (mover.ScreenPointInsideUnitWorldSpace(new(mouseX, mouseY)))
                {
                    unit = mover;
                    break;
                }
            }

            if (unit == null)
                foreach (var producer in Environment.Instance.Producers.GetNearbyProducers(pos))
                    if (producer.PositionInsideUnit(new(mouseX, mouseY)))
                    {
                        unit = producer;
                        break;
                    }

            if (unit == null)
            {
                Reset();
                return false;
            }
            else
            {
                var key = $"{ID}{((dynamic)unit).ID}";

                if (_selectedUnits.ContainsKey(key))
                {
                    _selectedUnits.TryRemove(key, out _);
                    Renderer.Instance.RemoveKeyContainedDrawCommand(key);
                }
                else
                {
                    _selectedUnits[key] = unit;
                }
                return true;
            }
        }

        /// <summary>
        /// Checks whether it’s time to update the selection state and refresh the UI if the number
        /// of selected units has changed.
        /// <list type="bullet">
        ///   <item><description>Exits early if the cycle isn’t running, a settings dialog is visible,
        ///     or the input threshold has not yet elapsed.</description></item>
        ///   <item><description>Updates the last input timestamp.</description></item>
        ///   <item><description>If the count of selected units differs from the previous count,
        ///     stores the new count and triggers a refresh.</description></item>
        /// </list>
        /// </summary>
        internal override bool LeftRelease()
        {
            if (!Cycle.IsRunning || UI.Instance.SettingPanel.Visible)
                return false;

            if (_currentCount != _selectedUnits.Count)
            {
                _currentCount = _selectedUnits.Count;

                foreach (var unit in _selectedUnits)
                    DrawStats(unit.Key, unit.Value);

                return true;
            }
            return false;
        }

        /// <summary>
        /// Re-draws the stats panel for the currently selected unit,
        /// if any.  This simply re-invokes <see cref="DrawStats(object,string)"/>
        /// using the last used draw ID.
        /// </summary>
        internal override void Render()
        {
            KeyValuePair<string, object>[] snapshot;
            lock (_lock)
            {
                snapshot = [.. _selectedUnits];
            }

            if (snapshot.Length == 0 || UI.Instance.SettingPanel.Visible)
            {
                foreach (var kv in snapshot)
                    Renderer.Instance.RemoveKeyContainedDrawCommand(kv.Key);
                return;
            }

            foreach (var kv in snapshot)
                DrawStats(kv.Key, kv.Value);
        }

        /// <summary>
        /// Removes all draw commands associated with the last stats panel.
        /// </summary>
        internal override void Remove()
        {
            string[] keys;
            lock (_lock)
            {
                keys = [.. _selectedUnits.Keys];
            }

            foreach (var key in keys)
                Renderer.Instance.RemoveKeyContainedDrawCommand(key);
        }

        /// <summary>
        /// Clears out any current selections and removes all draw commands
        /// associated with the last stats panel.
        /// </summary>
        internal override void Reset()
        {
            string[] keys;
            lock (_lock)
            {
                keys = [.. _selectedUnits.Keys];
                _selectedUnits.Clear();
            }

            foreach (var key in keys)
                Renderer.Instance.RemoveKeyContainedDrawCommand(key);
        }

        /// <summary>
        /// Renders a hover panel with detailed runtime statistics for a <c>Mover</c> or <c>Producer</c>.
        /// If a different unit is hovered than in the previous frame, the old panel is removed.
        /// <list type="bullet">
        ///   <item><description><b>ID and basic info:</b> Displays the unit's unique ID as a baseline identifier.</description></item>
        ///   <item><description><b>Mover details:</b> Shows traveled distance, total transports, position, velocity, acceleration, path length, product status, destination, and route distance.</description></item>
        ///   <item><description><b>Producer details:</b> Includes processer location, time spent queued, interaction counts, active product, queue size, and remaining processing ticks.</description></item>
        ///   <item><description><b>Panel layout:</b> Dynamically computes the bounding box size based on string widths, font metrics, and screen-space bounds.</description></item>
        ///   <item><description><b>Rendering:</b> Draws a gray rounded rectangle with aligned white text for label-value pairs using the Direct2D system.</description></item>
        /// </list>
        /// </summary>
        /// <param name="unitObj">The unit instance to inspect (must be a <c>Mover</c> or <c>Producer</c>).</param>
        /// <param name="id">Unique draw command key used to group and update the hover panel render commands.</param>
        private void DrawStats(string id, object unitObj)
        {
            var rows = new List<(string Label, string Value)>();
            var lines = new List<string>();

            var unit = (dynamic)unitObj;
            rows.Add(("ID", unit.ID));

            if (unit is Mover mover)
            {
                rows.Add(("Distance", $"{mover.Distance + mover.FractionalDistance:0.##} mm"));
                rows.Add(("Transport's", $"{mover.Count}"));
                rows.Add(("", ""));
                rows.Add(("Position", $"{mover.Center.X:0.##}  {mover.Center.Y:0.##}"));
                rows.Add(("Velocity", $"{mover.Velocity.X:0.##}  {mover.Velocity.Y:0.##}"));
                rows.Add(("Acceleration", $"{mover.Acceleration.X:0.##}  {mover.Acceleration.Y:0.##}"));
                rows.Add(("Path", $"{mover.Path.Count} steps"));
                rows.Add(("", ""));
                rows.Add(("Product", $"{(mover.ServiceRequester != ActorRefs.Nobody ? mover.ServiceRequester.Path.Name : "None")}"));
                var dest = Environment.Instance.Producers.Get().FirstOrDefault(p => p.Processer.Center == mover.Destination);
                rows.Add(("Destination", $"{mover.Destination.X:0.##}  {mover.Destination.Y:0.##}"));
                rows.Add(("Station", $"{dest?.ID ?? "Parking"}"));
                rows.Add(("Traveled", $"{mover.TransportDistance:0.##} mm"));
            }
            else if (unit is Producer producer)
            {
                rows.Add(("Processer", $"{producer.Processer.Center.X:0.##}  {producer.Processer.Center.Y:0.##}"));
                rows.Add(("Empty Queued", $"{producer.EmptyQueuedCounter} tick's"));
                rows.Add(("", ""));

                foreach (var kv in producer.InteractionCounter)
                {
                    var interaction = kv.Key;
                    var (count, ticks) = kv.Value;
                    rows.Add(("", $"{interaction}"));
                    rows.Add(("Performed", $"{count}"));
                    rows.Add(("Tick's", $"{ticks}"));
                    rows.Add(("", ""));
                }
                rows.Add(("Product", $"{(producer.ServiceRequester != ActorRefs.Nobody ? producer.ServiceRequester.Path.Name : "None")}"));
                rows.Add(("Queue Count", $"{producer.Queue.Count}"));
                rows.Add(("Processing", $"{producer.ProcessingCountdown} tick's"));
            }

            var widest = rows.Max(r => r.Label.Length);

            foreach (var (Label, Value) in rows)
            {
                if (string.IsNullOrEmpty(Label) && string.IsNullOrEmpty(Value))
                    lines.Add("");
                else if (string.IsNullOrEmpty(Label))
                    lines.Add(Value);
                else
                    lines.Add(Label.PadRight(widest) + Value);
            }

            var textStyle = TextStyles.Readable;

            var labelMaxWidth = rows
                .Where(r => !string.IsNullOrEmpty(r.Label))
                .Max(r => Renderer.Instance.GetTextLayout(r.Label, textStyle, padding: new(0, 0)).X) * 1.25f;

            var valueMaxWidth = rows
                .Where(r => !string.IsNullOrEmpty(r.Value) && !string.IsNullOrEmpty(r.Label))
                .Max(r => Renderer.Instance.GetTextLayout(r.Value, textStyle, padding: new(0, 0)).X);

            var sample = Renderer.Instance.GetTextLayout("Hg", textStyle, padding: new(0, 0));
            var lineH = sample.Y * 1.2f;
            var rowCount = rows.Count;

            var boxW = labelMaxWidth + valueMaxWidth;
            var boxH = rowCount * lineH;

            var center = Renderer.Instance.WorldToScreen(unit.Center);
            var pos = center - new Vector2(boxW / 2, boxH / 2);
            pos.X = Math.Clamp(pos.X, 0, Renderer.Instance.ScreenDimension.X - boxW);
            pos.Y = Math.Clamp(pos.Y, 0, Renderer.Instance.ScreenDimension.Y - boxH);

            Renderer.Instance.DrawRoundedRectangle(
                id,
                pos,
                new Vector2(boxW, boxH),
                _rectRadius * Renderer.Instance.ScaleUI,
                Colors.Gray85
            );

            for (var i = 0; i < rows.Count; i++)
            {
                var (label, value) = rows[i];
                var y = pos.Y + i * lineH;

                if (string.IsNullOrEmpty(label) && string.IsNullOrEmpty(value))
                    continue;

                if (string.IsNullOrEmpty(label))
                {
                    Renderer.Instance.DrawText(
                      id: $"{id}_sub_{i}",
                      text: value,
                      position: new Vector2(pos.X, y),
                      padding: new(0, 0),
                      style: textStyle,
                      color: Colors.White
                    );

                    continue;
                }

                Renderer.Instance.DrawText(
                  id: $"{id}_lbl_{i}",
                  text: label,
                  position: new Vector2(pos.X, y),
                  padding: new Vector2(0, 0),
                  style: textStyle,
                  color: Colors.White
                );

                Renderer.Instance.DrawText(
                  id: $"{id}_val_{i}",
                  text: value,
                  position: new Vector2(pos.X + labelMaxWidth, y),
                  padding: new Vector2(0, 0),
                  style: textStyle,
                  color: Colors.White
                );
            }
        }
    }
}
