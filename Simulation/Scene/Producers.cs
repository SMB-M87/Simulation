using System.Data;
using Akka.Actor;
using Interaction = Simulation.Unit.Interaction;
using KillProduct = Simulation.Dummy.KillProduct;
using Producer = Simulation.UnitProduction.Producer;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Manages all producer entities in the simulation, including initialization, grid registration and update logic.
    /// 
    /// This class tracks individual producers and their associated processing units,
    /// and ensures they are correctly placed within the simulation environment and forbidden zones.
    /// </summary>
    internal class Producers
    {
        private readonly HashSet<Producer> _producers = [];

        /// <summary>
        /// Initializes the <see cref="Producers"/> manager with a collection of producers and producer groups.
        /// Each producer is registered into spatial grids and forbidden zones.
        /// </summary>
        /// <param name="producers">A list of individual producers to add to the environment.</param>
        /// <param name="producerGroups">A list of grouped producers to be automatically laid out.</param>
        /// <param name="forbiddenZones">The forbidden zone registry for spatial collision prevention.</param>
        /// <param name="dimension">The simulation's overall space dimensions.</param>
        /// <param name="cellSize">The size of each grid cell used in spatial indexing.</param>
        internal Producers(
            List<Producer> producers,
            List<ProducerGroup> producerGroups,
            ForbiddenZones forbiddenZones,
            Vector2 dimension,
            Vector2 cellSize,
            uint maxQueue
            )
        {
            Addproducers(producers, forbiddenZones, dimension, cellSize);
            AddProducerGroups(producerGroups, forbiddenZones, dimension, cellSize);

            foreach (var producer in _producers)
                producer.MaxQueueCount = maxQueue;
        }

        /// <summary>
        /// Returns the set of all producers currently managed in the simulation.
        /// </summary>
        internal HashSet<Producer> Get()
        {
            return _producers;
        }

        /// <summary>
        /// Returns the producer matching the given id.
        /// </summary>
        internal Producer? Get(string id)
        {
            return _producers.FirstOrDefault(p => p.ID == id);
        }

        /// <summary>
        /// Returns the producer matching the given processing position.
        /// </summary>
        internal Producer? Get(Vector2 processing)
        {
            return _producers.FirstOrDefault(p => p.Processer.Center == processing);
        }

        /// <summary>
        /// Returns the set of producers that can process the given interaction.
        /// </summary>
        internal HashSet<Producer> Get(Interaction interaction)
        {
            return [.. _producers.Where(p => p.InteractionCost.ContainsKey(interaction))];
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Finds the producer assigned to the specified product actor name.</description></item>
        /// <item><description>Searches the <c>_movers</c> collection and returns the first producer whose <c>Product.Path.Name</c> matches the given string, or <c>null</c> if none match.</description></item>
        /// </list>
        /// </summary>
        internal Producer? GetByProduct(string product)
        {
            return _producers.FirstOrDefault(m => m.ServiceRequester != ActorRefs.Nobody && m.ServiceRequester.Path.Name == product);
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Returns a list of producers within range of the specified position.</description></item>
        /// <item><description>Calculates distance from each producer's center to the given position.</description></item>
        /// <item><description>Includes producers whose radius (scaled by 1.5) overlaps with the position.</description></item>
        /// </list>
        /// </summary>
        internal List<Producer> GetNearbyProducers(Vector2 position)
        {
            return [.. _producers.Where(p => (p.Center - position).Length() <= p.Radius * 1.5f)];
        }

        /// <summary>
        /// Determines whether any producer’s processing station falls within the specified grid cell.
        /// </summary>
        /// <param name="cell">
        ///   The target cell coordinates (X, Y) in world‐to‐grid space.
        /// </param>
        /// <returns>
        ///   <c>true</c> if at least one producer’s <see cref="Producer.Processer.Center"/>  
        ///   maps to the given <paramref name="cell"/>; otherwise, <c>false</c>.
        /// </returns>
        internal bool IsProcessingPosition((int X, int Y) cell)
        {
            return _producers.Any(p => Environment.Instance.GetCell(p.Processer.Center) == cell);
        }

        /// <summary>
        /// Updates each producer in the simulation.
        /// </summary>
        internal void Update()
        {
            foreach (var producer in _producers)
                producer.Update();
        }

        /// <summary>
        /// Resets all producers by clearing their queues, detaching assigned producers,
        /// and resetting their processing time.
        /// </summary>
        internal void Reset(bool dummy = false)
        {
            foreach (var producer in _producers)
            {
                if (dummy && producer.ServiceRequester != ActorRefs.Nobody)
                {
                    producer.ServiceRequester.Tell(new KillProduct());
                }
                producer.ResetQueue();
                producer.ServiceRequester = ActorRefs.Nobody;
                producer.ProcessingCountdown = 0;
                producer.RemoveProcessing();
            }
        }
        internal void FullReset(bool dummy = false)
        {
            foreach (var producer in _producers)
            {
                if (dummy && producer.ServiceRequester != ActorRefs.Nobody)
                {
                    producer.ServiceRequester.Tell(new KillProduct());
                }
                producer.ResetStats();
            }
        }

        /// <summary>
        /// Adds a list of individual producers to the simulation and registers them with spatial and forbidden zone logic.
        /// </summary>
        /// <param name="producers">The list of producers to add.</param>
        /// <param name="forbiddenZones">The forbidden zone manager used to prevent invalid placements.</param>
        /// <param name="dimension">The simulation area's full dimensions.</param>
        /// <param name="cellSize">The size of a single cell used for spatial partitioning.</param>
        private void Addproducers(
            List<Producer> producers,
            ForbiddenZones forbiddenZones,
            Vector2 dimension,
            Vector2 cellSize
            )
        {
            foreach (var producer in producers)
                AddingProducer(producer, forbiddenZones, dimension, cellSize);
        }

        /// <summary>
        /// Adds groups of producers arranged in a grid pattern based on position, dimension and spacing.
        /// Each producer is assigned a processing position and checked against forbidden zones before being added.
        /// </summary>
        /// <param name="producerGroups">The producer groups to add.</param>
        /// <param name="forbiddenZones">The forbidden zone manager used to prevent invalid placements.</param>
        /// <param name="Dimension">The simulation area's full dimensions.</param>
        /// <param name="cellSize">The size of a single cell used for spatial partitioning.</param>
        private void AddProducerGroups(
            List<ProducerGroup> producerGroups,
            ForbiddenZones forbiddenZones,
            Vector2 dimension,
            Vector2 cellSize
            )
        {
            foreach (var group in producerGroups)
            {
                var id = group.Id;
                var max = Math.Max(cellSize.X, cellSize.Y);
                var size = new Vector2(max, max);

                for (
                    var y = group.Position.Y;
                    y + size.Y + group.Spacing <= group.Position.Y + group.Dimension.Y;
                    y += size.Y + group.Spacing
                    )
                    for (
                        var x = group.Position.X;
                        x + size.X + group.Spacing <= group.Position.X + group.Dimension.X;
                        x += size.X + group.Spacing
                        )
                    {
                        var position = new Vector2(x, y);
                        var processingPos =
                            new Vector2(
                                group.ProcessingPos.X + (x - cellSize.X / 2),
                                group.ProcessingPos.Y + (y - cellSize.Y / 2)
                            );
                        var producer = new Producer(id++, group.Model, position, processingPos);

                        if (size == Vector2.Zero)
                            size = producer.ExtractStationaryDimension();

                        AddingProducer(producer, forbiddenZones, dimension, cellSize);
                    }
            }
        }

        /// <summary>
        /// Registers a single producer in the simulation, assigns its radius and processing area,
        /// and adds forbidden zones based on its size and position.
        /// </summary>
        /// <param name="producer">The producer instance to add.</param>
        /// <param name="forbiddenZones">The forbidden zone manager used to mark occupied grid cells.</param>
        /// <param name="dimension">The full simulation dimensions for assigning processor areas.</param>
        /// <param name="cellSize">The size of a single grid cell.</param>
        private void AddingProducer(
            Producer producer,
            ForbiddenZones forbiddenZones,
            Vector2 dimension,
            Vector2 cellSize
            )
        {
            producer.SetRadius(Math.Max(cellSize.X, cellSize.Y) / 2);
            producer.SetProcesser(dimension);

            _producers.Add(producer);

            var producerCell = (
                Math.Max(0, (int)(producer.Center.X / cellSize.X)),
                Math.Max(0, (int)(producer.Center.Y / cellSize.Y))
            );

            if (forbiddenZones.GetForbiddenZoneInCell(producerCell).Count > 0)
                return;

            var position = producer.ExtractStationaryPosition();
            var unitDimension = producer.ExtractStationaryDimension();

            if (unitDimension.X != cellSize.X || unitDimension.Y != cellSize.Y)
            {
                var id = 0;

                for (var y = position.Y; y < position.Y + unitDimension.Y; y += cellSize.Y)
                    for (var x = position.X; x < position.X + unitDimension.X; x += cellSize.X)
                        forbiddenZones.AddingForbiddenZone($"{producer.ID}_{id++}", new Vector2(x, y), cellSize, cellSize);
            }
            else
                forbiddenZones.AddingForbiddenZone($"{producer.ID}", position, unitDimension, cellSize);
        }
    }
}
