using Akka.Actor;
using Simulation.UnitProduction.Cost;
using UnitCircular = Simulation.Unit.UnitCircular;
using Interaction = Simulation.Unit.Interaction;
using Model = Simulation.Unit.Model;
using MQTTClient = Simulation.MQTT.Client;
using ProcessingCompleted = Simulation.Dummy.ProcessingCompleted;
using RectBody = Simulation.Unit.RectBody;
using SimColor = Simulation.Util.Color;
using State = Simulation.Unit.State;
using TextStyles = Simulation.Util.TextStyles;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UnitProduction
{
    /// <summary>
    /// Represents a production unit ("Producer") in the simulation. 
    /// Producers are responsible for processing items and interacting with movers through defined interaction points.
    /// Each producer has a model, a position, a processing station and may render in circular or rectangular form.
    /// </summary>
    /// <param name="id">A unique numeric ID used to identify this producer within its model group.</param>
    /// <param name="model">The model of the producer, which defines its interactions and colors.</param>
    /// <param name="position">The position of the producer in world coordinates.</param>
    /// <param name="processingPos">The position where processing effects are rendered.</param>
    /// <param name="rect">If <c>true</c>, the producer is rendered as a rectangle; otherwise, as a circle.</param>
    internal class Producer(
        int id,
        Model model,
        Vector2 position,
        Vector2 processingPos,
        bool rect = false
        ) : UnitCircular(
            $"{model}_{id}",
            model,
            position,
            ProducerModel.Specs[model].Color
            )
    {
        /// <summary>
        /// Used to calculate the estimated cost of a certain interaction.
        /// </summary>
        internal ProductionCost Cost { get; } = new LinearWeighted();

        /// <summary>
        /// Defines the processing characteristics for each supported <see cref="Interaction"/> type,
        /// based on this producer's model specification:
        /// <list type="bullet">
        ///   <item>
        ///     <description><c>Key</c>: The type of interaction (e.g., welding, painting).</description>
        ///   </item>
        ///   <item>
        ///     <description><c>Ticks</c>: Time required to complete the interaction (in ticks).</description>
        ///   </item>
        ///   <item>
        ///     <description><c>Cost</c>: Cost associated with performing the interaction.</description>
        ///   </item>
        /// </list>
        /// </summary>
        public readonly Dictionary<Interaction, (uint Ticks, uint Cost)> InteractionCost = ProducerModel.Specs[model].Interactions;

        /// <summary>
        /// Tracks per-interaction execution statistics for this producer:
        /// <list type="bullet">
        ///   <item>
        ///     <description>
        ///       <c>Key</c>: An <see cref="Interaction"/> type supported by the producer.
        ///     </description>
        ///   </item>
        ///   <item>
        ///     <description>
        ///       <c>Value.Executed</c>: Number of times this interaction has been executed (completed cycles).
        ///     </description>
        ///   </item>
        ///   <item>
        ///     <description>
        ///       <c>Value.Ticks</c>: Total number of simulation ticks spent processing this interaction type.
        ///     </description>
        ///   </item>
        /// </list>
        /// </summary>
        public readonly Dictionary<Interaction, (uint Executed, uint Ticks)> InteractionCounter =
            ProducerModel.Specs[model].Interactions.Keys.ToDictionary(
                 key => key,
                 value => (Executed: 0U, Ticks: 0U)
               );

        /// <summary>
        /// Queue of product IDs (as strings) waiting to be processed by this producer.
        /// </summary>
        internal List<string> Queue { get; private set; } = [];

        /// <summary>
        /// Max allowed queue count allowing to response to cost's proposal's.
        /// </summary>
        internal uint MaxQueueCount { get; set; }

        /// <summary>
        /// Gets the internal <see cref="Processer"/> instance responsible for managing processing state and position.
        /// </summary>
        internal RectBody Processer { get; } = new(processingPos, processingPos);

        /// <summary>
        /// Gets or sets the number of ticks remaining for the current processing operation.
        /// </summary>
        internal uint ProcessingCountdown { get; set; }

        /// <summary>
        /// Gets the number of updates representing one processing time unit.
        /// Based on the real mover max speed 2m/s and simulated mover max speed 5mm/tick 
        /// an estimated 400 tick's represents a second.
        /// </summary>
        internal uint ProcessingTimeUnit { get; } = 400;

        /// <summary>
        /// The interaction being actively processed.
        /// </summary>
        internal Interaction ProcessingInteraction { get; private set; }

        /// <summary>
        /// The number of simulation ticks where the producer had zero products waiting in its queue.
        /// Indicates potential underutilization or bottlenecks in assignment logic.
        /// </summary>
        internal ulong EmptyQueuedCounter { get; set; } = 0;

        /// <summary>
        /// The color used to visually indicate that this producer is actively processing.
        /// Defined by the model specification.
        /// </summary>
        private readonly Vector4 _processingColor = ProducerModel.Specs[model].ProcessingColor;

        /// <summary>
        /// The color used for rendering the ID text for this producer.
        /// Defined by the model specification.
        /// </summary>
        private readonly Vector4 _textColor = ProducerModel.Specs[model].TextColor;

        /// <summary>
        /// The size of the processing effect rendering area (e.g., for visual animation).
        /// </summary>
        private Vector2 _renderProcessingDimension;

        /// <summary>
        /// Whether this producer is rendered as a rectangle (true) or a circle (false).
        /// </summary>
        private readonly bool _renderRect = rect;

        /// <summary>
        /// Radius used for rendering the producer.
        /// </summary>
        private float _renderRadius;

        /// <summary>
        /// Sets the dimensions of the <see cref="Processer"/> for this producer.
        /// </summary>
        internal void SetProcesser(Vector2 Dimension) => Processer.Dimension = Dimension;

        /// <summary>
        /// Sets the collision and rendering radius for this producer.
        /// </summary>
        internal void SetRadius(float radius) => Radius = radius;

        /// <summary>
        /// Calculates the top-left position for rendering a rectangle or placing the producer in a static layout.
        /// </summary>
        internal Vector2 ExtractStationaryPosition()
        {
            var temp = Center;
            temp.X -= Radius;
            temp.Y -= Radius;
            return temp;
        }

        /// <summary>
        /// Gets the width and height of the producer assuming circular shape.
        /// </summary>
        internal Vector2 ExtractStationaryDimension()
        {
            var temp = Radius * 2;
            return new(temp, temp);
        }

        /// <summary>
        /// Gets the time (in ticks or cost units) associated with a specific interaction.
        /// </summary>
        /// <param name="interaction">The type of interaction.</param>
        /// <returns>Time required for the interaction, or int.MaxValue if unsupported.</returns>
        private uint GetTime(Interaction interaction)
        {
            if (!InteractionCost.TryGetValue(interaction, out var timeCost))
                return uint.MaxValue;

            return timeCost.Ticks;
        }

        /// <summary>
        /// Computes the cost of performing a specific interaction, taking into account the queue length and distance of the Dummy Procedure.
        /// </summary>
        /// <param name="interaction">The type of interaction.</param>
        /// <returns>Interaction cost as an integer.</returns>
        internal ulong GetDummyCost(Interaction interaction, ulong transportCost)
        {
            if (!InteractionCost.TryGetValue(interaction, out var cost) || Queue.Count >= MaxQueueCount)
                return ulong.MaxValue;

            return Cost.CalculateDummy(cost, Queue.Count, transportCost);
        }

        /// <summary>
        /// Computes the cost of performing a specific interaction, taking into account the queue length of the MQTT Procedure.
        /// </summary>
        /// <param name="interaction">The type of interaction.</param>
        /// <returns>Interaction cost as an integer.</returns>
        internal ulong GetMQTTCost(Interaction interaction)
        {
            if (!InteractionCost.TryGetValue(interaction, out var cost) || Queue.Count >= MaxQueueCount)
                return ulong.MaxValue;

            return Cost.CalculateMQTT(cost, Queue.Count);
        }

        /// <summary>
        /// Adds a mover ID to this producer’s processing queue.
        /// </summary>
        /// <param name="mover">The ID of the mover.</param>
        internal void AddQueue(string mover)
        {
            Queue.Add(mover);
        }

        /// <summary>
        /// Removes a mover ID from the queue.
        /// </summary>
        /// <param name="mover">The ID of the mover.</param>
        internal void RemoveQueue(string mover)
        {
            Queue.Remove(mover);
        }

        /// <summary>
        /// Clears the internal queue, removing all pending items or actions.
        /// </summary>
        internal void ResetQueue()
        {
            Queue = [];
        }

        /// <summary>
        /// Checks whether a given mover is already in the queue.
        /// </summary>
        internal bool QueueContains(string mover)
        {
            return Queue.Contains(mover);
        }

        /// <summary>
        /// Gets the priority (index) of a mover in the queue.
        /// </summary>
        /// <param name="mover">The ID of the mover.</param>
        /// <returns>Index in the queue, or -1 if not found.</returns>
        internal int GetPriority(string mover)
        {
            return Queue.IndexOf(mover);
        }

        /// <summary>
        /// Updates the rendering scale and offset for viewport transformations.
        /// </summary>
        /// <param name="scale">Current scale factor.</param>
        /// <param name="offset">Current screen offset.</param>
        internal void UpdateRendering(float scale)
        {
            _renderProcessingDimension = Processer.Dimension * scale;
            _renderRadius = Radius * scale;
        }

        /// <summary>
        /// Initiates a processing operation based on the specified interaction.
        /// Calculates the required processing time and sets it in simulation ticks.
        /// </summary>
        /// <param name="interaction">The interaction to be performed, used to determine processing time.</param>
        internal void StartProcessing(Interaction interaction, IActorRef actor)
        {
            ServiceRequester = actor;
            ProcessingCountdown = ProcessingTimeUnit * GetTime(interaction);
            ProcessingInteraction = interaction;
        }

        /// <summary>
        /// Completes the current processing operation and clears the associated product.
        /// Resets the internal actor reference to indicate no active processing.
        /// </summary>
        internal void CompleteProcessing()
        {
            ServiceRequester = ActorRefs.Nobody;
        }

        /// <summary>
        /// Updates the producer's processing logic on each simulation tick.
        /// If processing is complete, notifies the product actor.
        /// </summary>
        /// <param name="ups">Updates per second.</param>
        internal void Update()
        {
            if (ProcessingCountdown > 0 && State == State.Alive)
            {
                ProcessingCountdown--;
                var (Executed, Ticks) = InteractionCounter[ProcessingInteraction];

                if (ProcessingCountdown == 0)
                {
                    var processingTicks = ProcessingTimeUnit * GetTime(ProcessingInteraction);
                    InteractionCounter[ProcessingInteraction] = (Executed + 1U, Ticks + processingTicks);
                    Renderer.Instance.RemoveDrawCommand($"97_{ID}_0");

                    var mover = Environment.Instance.Movers.GetByProduct(ServiceRequester.Path.Name);
                    mover?.InteractionCompleted(processingTicks);

                    if (UI.Instance.SettingPanel.MQTT.Active)
                        MQTTClient.Instance.PublishComplete(ID);
                    else
                        ServiceRequester.Tell(new ProcessingCompleted(processingTicks));

                    RemoveQueue(ServiceRequester.Path.Name);
                    ServiceRequester = ActorRefs.Nobody;
                }
            }

            if (Queue.Count <= 0)
                EmptyQueuedCounter++;
        }

        /// <summary>
        /// Renders the producer using Direct2D based on its current shape (circle or rectangle),
        /// processing status and text labels.
        /// </summary>
        internal void Render()
        {
            if (_renderRect)
                DrawRectangularStation();
            else
                DrawCircularStation();

            DrawID();
            DrawProcessing();
        }

        private void DrawRectangularStation()
        {
            Renderer.Instance.DrawRectangle(
                id: $"4_{ID}_0",
                position: Renderer.Instance.WorldToScreen(Center - new Vector2(Radius, Radius)),
                dimension: new Vector2(_renderRadius * 2, _renderRadius * 2),
                color: State == State.Alive ? Color : SimColor.Red75
            );
        }

        private void DrawCircularStation()
        {
            Renderer.Instance.DrawCircle(
                id: $"4_{ID}_0",
                position: Renderer.Instance.WorldToScreen(Center),
                radius: _renderRadius,
                color: State == State.Alive ? Color : SimColor.Red75
            );
        }

        private void DrawID()
        {
            Renderer.Instance.DrawText(
                id: $"9_{ID}_label",
                text: ID,
                position: Renderer.Instance.WorldToScreen(Center),
                padding: new(0, 0),
                style: TextStyles.Readable,
                color: _textColor,
                center: true,
                UI: false
            );
        }

        private void DrawProcessing()
        {
            Renderer.Instance.DrawRectangle(
                 id: $"5_{ID}_2",
                 position: Renderer.Instance.WorldToScreen(Processer.Position),
                 dimension: _renderProcessingDimension,
                 color: _processingColor
             );
        }

        /// <summary>
        /// Renders the animated indicator for active processing (e.g., pulsing circle).
        /// </summary>
        internal void RenderProcessing()
        {
            if (ProcessingCountdown > 0 && State == State.Alive)
            {
                var renderProcessingPosition = Renderer.Instance.WorldToScreen(Processer.Center);

                Renderer.Instance.DrawCircle(
                    id: $"97_{ID}_0",
                    position: renderProcessingPosition,
                    radius: _renderRadius * 0.25f,
                    color: Color
                    );
            }
        }

        /// <summary>
        /// Removes the processing-related draw command associated with this entity from the rendering queue.
        /// </summary>
        internal void RemoveProcessing()
        {
            Renderer.Instance.RemoveDrawCommand($"97_{ID}_0");
        }

        /// <summary>
        /// Determines whether the given mouse position intersects this unit's rendered area.
        /// Handles both circular and rectangular visual representations.
        /// <list type="bullet">
        ///   <item>
        ///     <description>If <c>_renderRect</c> is true, treats the unit as a rectangle and checks bounds.</description>
        ///   </item>
        ///   <item>
        ///     <description>If <c>_renderRect</c> is false, treats the unit as a circle and checks radial distance from center.</description>
        ///   </item>
        /// </list>
        /// </summary>
        /// <param name="mouseX">Mouse X coordinate in screen space.</param>
        /// <param name="mouseY">Mouse Y coordinate in screen space.</param>
        /// <returns><c>true</c> if the point is inside the unit's rendered shape; otherwise, <c>false</c>.</returns>
        internal bool PositionInsideUnit(Vector2 position)
        {
            if (_renderRect)
            {
                var topLeft = Renderer.Instance.WorldToScreen((Center - new Vector2(Radius, Radius)));
                var size = new Vector2(Radius * 2, Radius * 2) * Renderer.Instance.Scale;

                if (RectBody.IsPointInsideRect(position, topLeft, size))
                    return true;
            }
            else
            {
                var center = Renderer.Instance.WorldToScreen(Center);
                var dist = Vector2.Distance(center, position);

                if (dist <= _renderRadius)
                    return true;
            }
            return false;
        }

        internal void ResetStats()
        {
            ResetQueue();
            ServiceRequester = ActorRefs.Nobody;
            ProcessingCountdown = 0;
            EmptyQueuedCounter = 0;
            RemoveProcessing();

            foreach (var interaction in InteractionCounter.Keys)
                InteractionCounter[interaction] = (0, 0);
        }
    }
}
