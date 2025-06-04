namespace Simulation.UnitTransport.Pathfinding
{
    /// <summary>
    /// Represents a cell node in the A* pathfinding graph.
    /// Contains state for pathfinding algorithms including heuristics, adjacency and path reconstruction.
    /// <list type="bullet">
    ///   <item><description><b>Cell:</b> The grid position (X, Y) of this vertex.</description></item>
    ///   <item><description><b>G, H, F:</b> The cost values used in pathfinding: G (cost from start), H (heuristic to goal), and F (total cost).</description></item>
    ///   <item><description><b>Prev:</b> The previous cell in the computed path.</description></item>
    ///   <item><description><b>Steps:</b> The number of steps taken to reach this vertex.</description></item>
    ///   <item><description><b>StraightIndex, DiagonalIndex:</b> Track the number of adjacent straight or diagonal cells.</description></item>
    ///   <item><description><b>Straight, Diagonal:</b> Arrays containing adjacent straight and diagonal cells.</description></item>
    ///   <item><description><b>Reset:</b> Resets the vertex to its default state or with a specific weight.</description></item>
    ///   <item><description><b>Heuristic:</b> Calculates the heuristic value and updates the total cost (F).</description></item>
    /// </list>
    /// </summary>
    internal class Vertex
    {
        /// <summary>
        /// The grid position (X, Y) of this vertex.
        /// </summary>
        public (int X, int Y) Cell;

        /// <summary>
        /// The cost from the start node to this node.
        /// </summary>
        public float G, H, F, CellWeight;

        /// <summary>
        /// The previous cell in the computed path.
        /// </summary>
        public (int X, int Y) Prev;

        /// <summary>
        /// The number of steps taken to reach this cell.
        /// </summary>
        public int Steps;

        /// <summary>
        /// Number of adjacent straight cells.
        /// </summary>
        public int StraightIndex;

        /// <summary>
        /// Number of adjacent diagonal cells.
        /// </summary>
        public int DiagonalIndex;

        /// <summary>
        /// Adjacent straight (orthogonal) cells.
        /// </summary>
        public (int X, int Y)[] Straight;

        /// <summary>
        /// Adjacent diagonal cells.
        /// </summary>
        public (int X, int Y)[] Diagonal;

        /// <summary>
        /// Creates a new uninitialized vertex with default values.
        /// </summary>
        /// <param name="count">The initial cell weight.</param>
        internal Vertex(int count)
        {
            Cell = (-1, -1);
            Prev = (-1, -1);
            Steps = 0;

            G = float.MaxValue;
            H = 0;
            F = float.MaxValue;
            CellWeight = count;

            Straight = new (int X, int Y)[4];
            Diagonal = new (int X, int Y)[4];

            StraightIndex = 0;
            DiagonalIndex = 0;
        }

        /// <summary>
        /// Creates a vertex with the specified cell position and weight.
        /// </summary>
        internal Vertex((int X, int Y) cell, int count) : this(count)
        {
            Cell = cell;
        }

        /// <summary>
        /// Resets the vertex to its default state.
        /// </summary>
        internal void Reset()
        {
            Prev = (-1, -1);
            Steps = 0;

            G = float.MaxValue;
            H = 0;
            F = float.MaxValue;
            CellWeight = 0;
        }

        /// <summary>
        /// Resets the vertex with a specified cell weight.
        /// </summary>
        internal void Reset(uint cellWeight, uint agentWeight)
        {
            Prev = (-1, -1);
            Steps = 0;

            G = float.MaxValue;
            H = 0;
            F = float.MaxValue;

            CellWeight = cellWeight - agentWeight;
        }

        /// <summary>
        /// Adds an adjacent cell to the vertex (either diagonal or straight).
        /// </summary>
        /// <param name="source">The source vertex.</param>
        /// <param name="dest">The destination cell coordinates.</param>
        internal static void AddAdjacent(Vertex source, (int X, int Y) dest)
        {
            if (source.Cell.X != dest.X && source.Cell.Y != dest.Y)
            {
                source.Diagonal[source.DiagonalIndex] = new(dest.X, dest.Y);
                source.DiagonalIndex++;
            }
            else
            {
                source.Straight[source.StraightIndex] = new(dest.X, dest.Y);
                source.StraightIndex++;
            }
        }

        /// <summary>
        /// Calculates the heuristic value (H) and updates the total cost (F).
        /// </summary>
        /// <param name="goal">The target cell.</param>
        /// <returns>The updated F score.</returns>
        internal float Heuristic((int X, int Y) goal)
        {
            var rowDiff = Math.Abs(Cell.X - goal.X);
            var colDiff = Math.Abs(Cell.Y - goal.Y);

            H = 1.41f * (rowDiff + colDiff) + (1.0f - 2 * 1.41f) * Math.Min(rowDiff, colDiff);
            F = G + H;
            return F;
        }

        /// <summary>
        /// Provides a shared Null vertex instance.
        /// </summary>
        internal static Vertex Null { get; } = new Vertex(-1);
    }
}
