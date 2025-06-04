using Simulation.UnitTransport.Pathfinding;
using NavigableGrid = Simulation.Scene.NavigableGrid;

namespace Simulation.UnitTransport.NavComposite
{
    /// <summary>
    /// Implements A* pathfinding for movable ctx.Agents on a navigable grid.
    /// Calculates a path from the ctx.Agent's current position to its destination,
    /// and stores the resulting waypoints in the ctx.Agent's path.
    /// <list type="bullet">
    ///   <item><description><b>grid:</b> The navigable grid which contains cell data and layout information. This grid is used to build the pathfinding graph and perform the A* search.</description></item>
    ///   <item><description><b>ctx.Agent:</b> The movable ctx.Agent that needs to find a path. This parameter is used to get the ctx.Agent's current position and destination, and to update its path after the search is complete.</description></item>
    ///   <item><description><b>ctx.Environment:</b> The navigable ctx.Environment that provides information about the grid and the size of each cell. This parameter is used in calculating the path and converting the waypoints from grid cells to world positions.</description></item>
    /// </list>
    /// </summary>
    /// <param name="grid">Represent all navigable grid cell's of the ctx.Environment as value of the key value pair is the associated cellweight.</param>
    internal class AStar(Dictionary<(int X, int Y), uint> grid) : Algorithm
    {
        /// <summary>
        /// Initializes the graph used for pathfinding in the A* algorithm.
        /// <list type="bullet">
        ///   <item><description><b>Input:</b> A dictionary representing the grid where each cell is mapped to its corresponding weight.</description></item>
        ///   <item><description><b>Logic:</b> The graph is constructed based on the provided grid. The grid consists of cell coordinates as keys and their respective weights as values. These weights are used to define the cost of traversing each cell in the ctx.Environment.</description></item>
        ///   <item><description><b>Output:</b> A new instance of the <see cref="Graph"/> class is created, which will be used for pathfinding calculations during the A* algorithm.</description></item>
        /// </list>
        /// </summary>
        private readonly Graph _graph = new(grid);

        /// <summary>
        /// Entry point to trigger pathfinding.
        /// Only calculates a path if the ctx.Agent doesn't already have one
        /// and if the destination is far enough to require navigation.
        /// <list type="bullet">
        ///   <item><description><b>ctx.Agent:</b> The ctx.Agent that needs a path. The ctx.Agent's current position and destination are used to calculate the path, and the resulting waypoints are stored in the ctx.Agent's Path property.</description></item>
        ///   <item><description><b>ctx.Environment:</b> The navigable grid (cell layout). This provides the grid size and information about the ctx.Environment to perform the A* pathfinding algorithm and calculate a path from the ctx.Agent's current position to the destination.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx">The current navigation context containing agent and environment data.</param>
        internal override void FindPath(NavigationContext ctx)
        {
            if (ctx.Agent.Path.Count == 0 ||
                (ctx.Agent.Path.Count > 0 &&
                (ctx.Agent.Center - ctx.Agent.Path.Peek()).Length() >= ctx.Environment.CellSize.Length()))
                CalculatePath(ctx.Agent, ctx.Environment);
        }

        /// <summary>
        /// Performs A* search on the graph from the ctx.Agent's current cell to the destination cell.
        /// Populates the ctx.Agent's path with the result, translated from cells to actual world positions.
        /// <list type="bullet">
        ///   <item><description><b>ctx.Agent:</b> The ctx.Agent to find a path for. The ctx.Agent's current position and destination are used to calculate the path, and the resulting waypoints are stored in the ctx.Agent's Path property.</description></item>
        ///   <item><description><b>ctx.Environment:</b> The grid ctx.Environment containing cell data and size. This provides the necessary grid information and cell size used to calculate the path and translate grid cells into world positions.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx.Agent">The ctx.Agent to find a path for.</param>
        /// <param name="ctx.Environment">The grid ctx.Environment containing cell data and size.</param>
        private void CalculatePath(MovableBody agent, NavigableGrid environment)
        {
            var currentCell = environment.GetCell(agent.Center);
            var destCell = environment.GetCell(agent.Destination);

            if (!environment.Grid.ContainsKey(currentCell) ||
                !environment.Grid.ContainsKey(destCell))
                return;

            _graph.ClearAll();

            if (_graph.AStar(destCell, currentCell, out var output) && output != null)
            {
                agent.Path =
                    _graph.GetPathBezier(
                        output,
                        environment.CellSize,
                        agent.Destination,
                        agent.Center
                        );
            }
        }

        /// <summary>
        /// Returns an identifier for the A* pathfinding algorithm.
        /// This method can be used for debugging or logging purposes to identify the algorithm in use.
        /// <list type="bullet">
        ///   <item><description><b>Output:</b> The string "AStar", representing the unique identifier for the A* algorithm.</description></item>
        /// </list>
        /// </summary>
        internal override string GetID()
        {
            return "AStar";
        }
    }
}
