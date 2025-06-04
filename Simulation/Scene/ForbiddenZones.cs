using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Manages all forbidden zones in the simulation, including spatial indexing,
    /// zone queries and dynamic forbidden area registration.
    /// 
    /// Forbidden zones define areas that movers are not allowed to enter.
    /// </summary>
    internal class ForbiddenZones
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_forbiddenZones</b>: Stores all defined forbidden zones in the environment.</description></item>
        /// </list>
        /// </summary>
        private readonly HashSet<ForbiddenZone> _forbiddenZones = [];

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_forbiddenZoneGrid</b>: Maps grid cell positions to sets of forbidden zone IDs occupying those cells.</description></item>
        /// </list>
        /// </summary>
        private readonly Dictionary<(int, int), HashSet<string>> _forbiddenZoneGrid = [];

        /// <summary>
        /// Initializes the <see cref="ForbiddenZones"/> manager with a collection of predefined forbidden zones.
        /// Zones are subdivided and registered into spatial grid cells for fast lookup.
        /// </summary>
        /// <param name="forbiddenZones">The list of forbidden zones to add to the simulation.</param>
        /// <param name="cellSize">The size of each spatial grid cell.</param>
        internal ForbiddenZones(List<ForbiddenZone> forbiddenZones, Vector2 cellSize)
        {
            AddforbiddenZones(forbiddenZones, cellSize);
        }

        /// <summary>
        /// Returns the complete set of forbidden zones currently managed by the simulation.
        /// </summary>
        internal HashSet<ForbiddenZone> Get()
        {
            return _forbiddenZones;
        }

        /// <summary>
        /// Gets all forbidden zone IDs that are registered in the given grid cell.
        /// </summary>
        /// <param name="cell">The (x, y) coordinate of the grid cell.</param>
        internal HashSet<string> GetForbiddenZoneInCell((int, int) cell)
        {
            return _forbiddenZoneGrid.TryGetValue(cell, out var zone) ? zone : [];
        }

        /// <summary>
        /// Returns all cells within the simulation grid that contain one or more forbidden zones.
        /// </summary>
        /// <param name="dimension">The simulation area's dimensions.</param>
        /// <param name="cellSize">The size of a single spatial cell.</param>
        internal HashSet<(int, int)> GetAllForbiddenZoneCells()
        {
            return [.. _forbiddenZoneGrid
                .Where(kvp => kvp.Value.Count > 0)
                .Select(kvp => kvp.Key)];
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>AddforbiddenZones</b>: Adds forbidden zones to the environment based on their dimensions and cell size.</description></item>
        /// <item><description>Iterates through each <b>ForbiddenZone</b> in the list.</description></item>
        /// <item><description>If zone dimensions differ from cell size, subdivides the zone into smaller forbidden cells.</description></item>
        /// <item><description>Generates a unique ID for each sub-cell and calls <b>AddingForbiddenZone</b> per cell.</description></item>
        /// <item><description>If dimensions match cell size, adds the zone directly as a single cell.</description></item>
        /// </list>
        /// </summary>
        private void AddforbiddenZones(List<ForbiddenZone> forbiddenZones, Vector2 cellSize)
        {
            foreach (var noGoZone in forbiddenZones)
                if (noGoZone.Dimension.X != cellSize.X || noGoZone.Dimension.Y != cellSize.Y)
                {
                    var id = 0;

                    for (
                        var y = noGoZone.Position.Y;
                        y < noGoZone.Position.Y + noGoZone.Dimension.Y;
                        y += cellSize.Y
                        )
                        for (
                            var x = noGoZone.Position.X;
                            x < noGoZone.Position.X + noGoZone.Dimension.X;
                            x += cellSize.X
                            )
                            AddingForbiddenZone($"{noGoZone.ID}_{id++}", new Vector2(x, y), cellSize, cellSize);
                }
                else
                    AddingForbiddenZone(noGoZone.ID, noGoZone.Position, noGoZone.Dimension, cellSize);
        }

        /// <summary>
        /// Registers a single forbidden zone by ID and inserts it into the correct grid cell.
        /// </summary>
        /// <param name="name">The unique identifier of the forbidden zone.</param>
        /// <param name="position">The top-left position of the zone.</param>
        /// <param name="dimension">The width and height of the zone.</param>
        /// <param name="cellSize">The grid size to determine cell registration.</param>
        internal void AddingForbiddenZone(
            string name,
            Vector2 position,
            Vector2 dimension,
            Vector2 cellSize
            )
        {
            var forbiddenZone = new ForbiddenZone(
                name,
                position,
                dimension
                );

            _forbiddenZones.Add(forbiddenZone);

            var cell = (
                Math.Max(0, (int)(forbiddenZone.Center.X / cellSize.X)),
                Math.Max(0, (int)(forbiddenZone.Center.Y / cellSize.Y))
            );

            if (!_forbiddenZoneGrid.TryGetValue(cell, out var forbiddenZoneCell))
            {
                forbiddenZoneCell = [];
                _forbiddenZoneGrid[cell] = forbiddenZoneCell;
            }
            forbiddenZoneCell.Add(name);
        }
    }
}
