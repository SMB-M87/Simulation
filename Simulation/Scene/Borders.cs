using MovableBody = Simulation.UnitTransport.MovableBody;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.Scene
{
    /// <summary>
    /// Represents the border segments of a grid-based environment, used for detecting potential collisions.
    /// Borders are generated for grid cells that do not have adjacent neighbors in the cardinal directions.
    /// </summary>
    internal class Borders
    {
        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>_borders</b>: Maps grid cell coordinates to a list of border segments, each defined by a start and end <b>Vector2</b> point.</description></item>
        /// </list>
        /// </summary>
        private readonly Dictionary<(int X, int Y), List<(Vector2 A, Vector2 B)>> _borders = [];

        /// <summary>
        /// Constructs the border map from the given grid layout and cell size.
        /// For each cell, it adds borders on the sides where neighboring cells are missing.
        /// </summary>
        /// <param name="grid">A dictionary of grid cells to values (value is unused here).</param>
        /// <param name="cellSize">The width and height of each grid cell.</param>
        internal Borders(Dictionary<(int X, int Y), uint> grid, Vector2 cellSize)
        {
            foreach (var cell in grid.Keys)
            {
                var borders = new List<(Vector2, Vector2)>();

                if (!grid.ContainsKey((cell.X - 1, cell.Y)))
                {
                    var border =
                        (new Vector2(cellSize.X * cell.X, cellSize.Y * cell.Y),
                         new Vector2(cellSize.X * cell.X, cellSize.Y * cell.Y + cellSize.Y));

                    borders.Add(border);
                }

                if (!grid.ContainsKey((cell.X + 1, cell.Y)))
                {
                    var border =
                        (new Vector2(cellSize.X * cell.X + cellSize.X, cellSize.Y * cell.Y),
                         new Vector2(cellSize.X * cell.X + cellSize.X, cellSize.Y * cell.Y + cellSize.Y));

                    borders.Add(border);
                }

                if (!grid.ContainsKey((cell.X, cell.Y - 1)))
                {
                    var border =
                        (new Vector2(cellSize.X * cell.X, cellSize.Y * cell.Y),
                         new Vector2(cellSize.X * cell.X + cellSize.X, cellSize.Y * cell.Y));

                    borders.Add(border);
                }

                if (!grid.ContainsKey((cell.X, cell.Y + 1)))
                {
                    var border =
                        (new Vector2(cellSize.X * cell.X, cellSize.Y * cell.Y + cellSize.Y),
                         new Vector2(cellSize.X * cell.X + cellSize.X, cellSize.Y * cell.Y + cellSize.Y));

                    borders.Add(border);
                }

                if (borders.Count > 0)
                    _borders[cell] = borders;
            }
        }

        /// <summary>
        /// Returns all the borders stored in the environment.
        /// </summary>
        internal Dictionary<(int X, int Y), List<(Vector2 A, Vector2 B)>> Get()
        {
            return _borders;
        }

        /// <summary>
        /// Returns all possible border segments that a given agent could collide with.
        /// Checks the surrounding 3x3 cells around the agent's current position.
        /// </summary>
        /// <param name="agent">The movable body for which to check potential collisions.</param>
        /// <returns>List of potential border segments near the agent.</returns>
        internal List<(Vector2 A, Vector2 B)> GetPossibleCollisions(MovableBody agent)
        {
            List<(Vector2 A, Vector2 B)> borders = [];

            if (agent == null)
                return borders;

            var (X, Y) = Environment.Instance.GetCell(agent.Center);

            for (var x = -1; x <= 1; x++)
                for (var y = -1; y <= 1; y++)
                    if (_borders.TryGetValue((X + x, Y + y), out var bordersInCell))
                        foreach (var border in bordersInCell)
                            borders.Add(border);

            return borders;
        }
    }
}
