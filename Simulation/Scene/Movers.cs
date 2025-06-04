using Akka.Actor;
using ActorRefs = Akka.Actor.ActorRefs;
using KillProduct = Simulation.Dummy.KillProduct;
using MovableBody = Simulation.UnitTransport.MovableBody;
using Mover = Simulation.UnitTransport.Mover;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Manages all transport units ("movers") in the simulation, including registration, spatial partitioning,
    /// collision estimation, navigation setup and periodic updates.
    /// 
    /// Maintains a spatial hash grid for efficient proximity queries and pathfinding support,
    /// and provides APIs for interacting with, resetting, or debugging all movers.
    /// </summary>
    internal class Movers
    {
        /// <summary>
        /// Set containing all registered movers in the simulation.
        /// Used for global iteration and updates.
        /// </summary>
        private readonly HashSet<Mover> _movers = [];

        /// <summary>
        /// Lookup table mapping mover IDs to their corresponding <see cref="Mover"/> instances.
        /// Enables fast access by ID.
        /// </summary>
        private readonly Dictionary<string, Mover> _moverLookup = [];

        /// <summary>
        /// Spatial grid mapping cell coordinates to the set of mover IDs currently occupying each cell.
        /// Used for efficient neighborhood and collision queries.
        /// </summary>
        private readonly Dictionary<(int, int), HashSet<string>> _moverGrid = [];

        /// <summary>
        /// Initializes a new instance of the <see cref="Movers"/> class with a given list of movers and groups.
        /// </summary>
        /// <param name="movers">A flat list of movers to add to the simulation.</param>
        /// <param name="moverGroups">A list of grouped movers to auto-generate in a spatial grid.</param>
        /// <param name="cellSize">The grid cell size used for spatial partitioning.</param>
        internal Movers(
            List<Mover> movers,
            List<MoverGroup> moverGroups,
            Vector2 cellSize
            )
        {
            AddMovers(movers, cellSize);
            AddMoverGroups(moverGroups, cellSize);
        }

        /// <summary>
        /// Initializes navigation behavior for all movers by providing them with a grid-based navigation map.
        /// Each mover will configure its pathfinding logic based on the given grid layout.
        /// </summary>
        /// <param name="grid">A dictionary representing the navigation grid where keys are cell coordinates and values define cell cost or type.</param>
        internal void InitNavigation(Dictionary<(int, int), uint> grid)
        {
            foreach (var mover in _movers)
                mover.Setup(grid);
        }

        /// <summary>
        /// Returns the complete set of registered movers in the simulation.
        /// </summary>
        internal HashSet<Mover> Get()
        {
            return _movers;
        }

        /// <summary>
        /// Retrieves a mover by its unique ID.
        /// </summary>
        /// <param name="id">The identifier of the mover.</param>
        /// <returns>The corresponding <see cref="Mover"/>, or <c>null</c> if not found.</returns>
        internal Mover? Get(string id)
        {
            return _moverLookup.TryGetValue(id, out var mover) ? mover : null;
        }

        /// <summary>
        /// Retrieves a mover by destination.
        /// </summary>
        /// <param name="id">The identifier of the mover.</param>
        /// <returns>The corresponding <see cref="Mover"/>, or <c>null</c> if not found.</returns>
        internal Mover? Get(Vector2 destination)
        {
            return _movers.FirstOrDefault(m => m.Destination == destination || m.SwapDestination == destination);
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description>Finds the mover assigned to the specified product actor name.</description></item>
        /// <item><description>Searches the <c>_movers</c> collection and returns the first mover whose <c>Product.Path.Name</c> matches the given string, or <c>null</c> if none match.</description></item>
        /// </list>
        /// </summary>
        internal Mover? GetByProduct(string product)
        {
            return _movers.FirstOrDefault(m => m.ServiceRequester != ActorRefs.Nobody && m.ServiceRequester.Path.Name == product);
        }

        /// <summary>
        /// Retrieves nearby <see cref="MovableBody"/> instances that may collide with the given agent,
        /// based on the agent's current spatial cell and its neighbors.
        /// Results are sorted by effective proximity (distance minus combined radii).
        /// </summary>
        /// <param name="agent">The mover for which to find possible collisions.</param>
        /// <returns>List of nearby physics bodies, sorted by proximity.</returns>
        internal List<MovableBody> GetPossibleCollisions(MovableBody agent)
        {
            List<MovableBody> possibleCollisions = [];

            if (agent == null)
                return possibleCollisions;

            var (X, Y) = Environment.Instance.GetCell(agent.Center);

            for (var x = -1; x <= 1; x++)
                for (var y = -1; y <= 1; y++)
                    foreach (var other in GetMoversInCell((X + x, Y + y)))
                        if (_moverLookup.TryGetValue(other, out var otherMover) && otherMover.ID != agent.ID)
                            possibleCollisions.Add(otherMover);

            possibleCollisions.Sort((a, b) =>
            {
                var distanceA = Vector2.Distance(a.Center, agent.Center);
                var distanceB = Vector2.Distance(b.Center, agent.Center);

                var effectiveA = distanceA - (agent.Radius + a.Radius);
                var effectiveB = distanceB - (agent.Radius + b.Radius);

                return effectiveA.CompareTo(effectiveB);
            });

            return possibleCollisions;
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>GetNearbyMovers</b>: Returns a list of movers near the specified position for collision detection.</description></item>
        /// <item><description>Determines the current grid cell of the given position.</description></item>
        /// <item><description>Checks the 3×3 surrounding grid cells for nearby movers.</description></item>
        /// <item><description>Filters and adds valid movers from the lookup dictionary to the result list.</description></item>
        /// </list>
        /// </summary>
        internal List<Mover> GetNearbyMovers(Vector2 position)
        {
            return [.. _movers.Where(p => (p.Center - position).Length() <= p.Radius * 1.5f)];
        }

        /// <summary>
        /// Calculates the total number of movers occupying each cell in the specified grid.
        /// Used for estimating spatial load and influencing pathfinding decisions.
        /// </summary>
        /// <param name="grid">The grid structure over which to count movers.</param>
        /// <returns>A dictionary mapping each grid cell to its aggregated mover weight.</returns>
        internal Dictionary<(int, int), uint> GetAllCellWeights(NavigableGrid grid)
        {
            var cellCounts = new Dictionary<(int, int), uint>();

            foreach (var kvp in grid.Grid)
                cellCounts[kvp.Key] = 0;

            foreach (var kvp in grid.Grid)
                if (_moverGrid.TryGetValue(kvp.Key, out var movers))
                {
                    foreach (var mover in movers)
                    {
                        if (_moverLookup.TryGetValue(mover, out var entity))
                        {
                            var topLeftCorner = entity.Position;
                            var topRightCorner = topLeftCorner;
                            topRightCorner.X += entity.Dimension.X;

                            var bottomRightCorner = entity.Position + entity.Dimension;
                            var bottomLeftCorner = bottomRightCorner;
                            bottomLeftCorner.X -= entity.Dimension.X;

                            var cornerCells = new[]
                            {
                                grid.GetCell(topLeftCorner),
                                grid.GetCell(topRightCorner),
                                grid.GetCell(bottomRightCorner),
                                grid.GetCell(bottomLeftCorner)
                            };

                            var delta = entity.CellWeight / 4;

                            foreach (var cell in cornerCells)
                                if (cellCounts.ContainsKey(cell))
                                    cellCounts[cell] += delta;
                        }
                    }
                }

            return cellCounts;
        }

        /// <summary>
        /// Updates a mover's location in the spatial grid. Removes it from its old cell and registers it in the new cell.
        /// Also updates the cell weight contributions based on its current and previous positions.
        /// </summary>
        /// <param name="moverId">The unique ID of the mover.</param>
        /// <param name="oldPos">The mover's previous position.</param>
        /// <param name="newPos">The mover's current position.</param>
        /// <param name="dimension">The mover's dimensions for grid projection.</param>
        /// <param name="cellWeight">The mover's weight contribution to the grid.</param>
        internal void UpdateMoverPosition(
            string moverId,
            Vector2 oldPos,
            Vector2 newPos,
            Vector2 dimension,
            uint cellWeight
            )
        {
            var previous = Environment.Instance.GetCell(oldPos);
            var current = Environment.Instance.GetCell(newPos);

            UpdateCellWeight(oldPos, dimension, cellWeight);
            UpdateCellWeight(newPos, dimension, cellWeight, false);

            if (previous == current || !_moverLookup.ContainsKey(moverId))
                return;

            if (_moverGrid.TryGetValue(previous, out var oldCell))
            {
                oldCell.Remove(moverId);

                if (oldCell.Count == 0)
                    _moverGrid.Remove(previous);
            }

            if (!_moverGrid.TryGetValue(current, out var newCell))
            {
                newCell = [];
                _moverGrid[current] = newCell;
            }
            newCell.Add(moverId);
        }

        /// <summary>
        /// Resets all movers to a clean state by clearing their paths, resetting confirmation flags,
        /// and detaching any assigned producers. In dummy mode, also sends a <see cref="KillProduct"/> message.
        /// </summary>
        /// <param name="dummy">If <c>true</c>, triggers actor shutdown for attached producers.</param>
        internal void Reset(bool dummy = false)
        {
            foreach (var mover in _movers)
            {
                mover.Completed = false;

                if (dummy && mover.ServiceRequester != ActorRefs.Nobody)
                {
                    mover.ServiceRequester.Tell(new KillProduct());
                }
                else
                {
                    mover.ServiceRequester = ActorRefs.Nobody;

                    if (!Environment.Instance.Parkings.IsParkingSpace(mover.Destination))
                        mover.Destination = Vector2.Zero;
                }
                mover.Reset = true;
            }
        }
        internal void FullReset(bool dummy = false)
        {
            foreach (var mover in _movers)
            {
                mover.Completed = false;

                if (dummy && mover.ServiceRequester != ActorRefs.Nobody)
                {
                    mover.ServiceRequester.Tell(new KillProduct());
                }
                else
                {
                    mover.ServiceRequester = ActorRefs.Nobody;

                    if (!Environment.Instance.Parkings.IsParkingSpace(mover.Destination))
                        mover.Destination = Vector2.Zero;
                }
                mover.ResetStats();
            }
        }

        /// <summary>
        /// Updates the cell weights of a rectangular area in the spatial grid.
        /// Used to influence pathfinding algorithms or visualize load balancing.
        /// </summary>
        /// <param name="position">The center position of the area.</param>
        /// <param name="dimension">The area size.</param>
        /// <param name="cellWeight">The total weight to distribute across the area.</param>
        /// <param name="remove">If <c>true</c>, subtracts weight; otherwise, adds it.</param>
        private static void UpdateCellWeight(
            Vector2 position,
            Vector2 dimension,
            uint cellWeight,
            bool remove = true)
        {
            var topLeftCorner = position - dimension / 2;
            var topRightCorner = topLeftCorner;
            topRightCorner.X += dimension.X;

            var bottomRightCorner = topLeftCorner + dimension;
            var bottomLeftCorner = bottomRightCorner;
            bottomLeftCorner.X -= dimension.X;

            var cornerCells = new[]
            {
                Environment.Instance.GetCell(topLeftCorner),
                Environment.Instance.GetCell(topRightCorner),
                Environment.Instance.GetCell(bottomRightCorner),
                Environment.Instance.GetCell(bottomLeftCorner)
            };

            var delta = (uint)(remove ? -(cellWeight / 4) : cellWeight / 4);

            foreach (var cell in cornerCells)
                if (Environment.Instance.Grid.ContainsKey(cell))
                    Environment.Instance.Grid[cell] += delta;
        }

        /// <summary>
        /// Performs a simulation tick update on all movers.
        /// Refreshes the UI collision counter if any new collisions occurred during this frame.
        /// </summary>
        internal void Update()
        {
            var collision = Environment.Instance.Collisions;

            foreach (var mover in _movers)
                mover.Update();

            if (collision != Environment.Instance.Collisions)
                UI.Instance.SettingPanel.Counter.Interact();
        }

        /// <summary>
        /// Retrieves the set of mover IDs located in a specific spatial grid cell.
        /// </summary>
        /// <param name="cell">Grid cell coordinates.</param>
        /// <returns>A set of mover IDs in the cell, or an empty set if none exist.</returns>
        private HashSet<string> GetMoversInCell((int, int) cell)
        {
            return _moverGrid.TryGetValue(cell, out var movers) ? movers : [];
        }

        /// <summary>
        /// Adds a list of individual movers to the simulation and registers them in the spatial grid.
        /// </summary>
        /// <param name="movers">The list of movers to add.</param>
        /// <param name="cellSize">The size of each grid cell for partitioning.</param>
        private void AddMovers(List<Mover> movers, Vector2 cellSize)
        {
            foreach (var mover in movers)
                AddingMover(mover, cellSize);
        }

        /// <summary>
        /// Adds multiple mover groups to the simulation, automatically arranging movers in a spatial layout
        /// based on group size, spacing and position.
        /// </summary>
        /// <param name="moverGroups">The groups of movers to generate.</param>
        /// <param name="cellSize">The grid cell size for spatial alignment.</param>
        private void AddMoverGroups(List<MoverGroup> moverGroups, Vector2 cellSize)
        {
            foreach (var group in moverGroups)
            {
                var id = group.Id;
                var size = Vector2.Zero;

                for (
                    var y = group.Position.Y;
                    y + size.Y < group.Position.Y + group.Dimension.Y;
                    y += size.Y + group.Spacing
                     )
                    for (
                        var x = group.Position.X;
                        x + size.X < group.Position.X + group.Dimension.X;
                        x += size.X + group.Spacing
                        )
                    {
                        var mover = new Mover(id++, group.Model, new Vector2(x, y));

                        if (size == Vector2.Zero)
                            size = mover.Dimension;

                        AddingMover(mover, cellSize);
                    }
            }
        }

        /// <summary>
        /// Adds a single mover to the simulation, calculates its cell weight based on size,
        /// and registers it in the spatial lookup structures.
        /// </summary>
        /// <param name="mover">The mover to register.</param>
        /// <param name="cellSize">The size of a spatial grid cell.</param>
        private void AddingMover(Mover mover, Vector2 cellSize)
        {
            var X = cellSize.X / mover.Dimension.X;
            var Y = cellSize.Y / mover.Dimension.Y;

            mover.CellWeight =
                (uint)(X < 2 && Y < 2 ?
                16 : X < 2 || Y < 2 ?
                8 : 4);

            if (_movers.Add(mover))
                _moverLookup[mover.ID] = mover;

            var cell = (
                Math.Max(0, (int)(mover.Center.X / cellSize.X)),
                Math.Max(0, (int)(mover.Center.Y / cellSize.Y))
            );

            if (!_moverGrid.TryGetValue(cell, out var moversInCell))
            {
                moversInCell = [];
                _moverGrid[cell] = moversInCell;
            }
            moversInCell.Add(mover.ID);
        }
    }
}
