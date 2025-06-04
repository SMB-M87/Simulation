using Simulation.UnitTransport.Pathfinding;
using NavigableGrid = Simulation.Scene.NavigableGrid;
using Vector2 = System.Numerics.Vector2;

namespace Simulation.UnitTransport.NavComposite
{
    /// <summary>
    /// An A* pathfinding algorithm that also considers a dynamic heatmap,
    /// allowing path recalculation when current or next cells become too "hot" (congested).
    /// </summary>
    internal class AStarHeatmap(Dictionary<(int X, int Y), uint> grid) : Algorithm
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

        private uint _counter = 0;

        /// <summary>
        /// Determines whether a new path needs to be calculated based on:
        /// 
        /// <list type="bullet">
        ///   <item><description>No path present.</description></item>
        ///   <item><description>Destination far enough.</description></item>
        ///   <item><description>Dynamic heatmap values of current and next cell.</description></item>
        /// </list>
        /// <list type="bullet">
        ///   <item><description><b>Inputs:</b>
        ///     <list type="bullet">
        ///       <item><description><b>ctx.Agent:</b> The ctx.Agent that requires navigation. Provides the current position, destination, and existing path information.</description></item>
        ///       <item><description><b>ctx.Environment:</b> The navigable grid representing the ctx.Environment, used to get cell information and perform pathfinding calculations.</description></item>
        ///     </list>
        ///   </description></item>
        ///   <item><description><b>Logic:</b>
        ///     Checks if a new path needs to be calculated based on the following conditions:
        ///     <list type="bullet">
        ///       <item><description>If the ctx.Agent doesn't already have a path and has a valid destination that is far enough away from the current position, it triggers the path calculation.</description></item>
        ///       <item><description>If the ctx.Agent already has a path, it checks the current and next cells to determine if dynamic heatmap values or other conditions require a new path to be calculated.</description></item>
        ///     </list>
        ///   </description></item>
        ///   <item><description><b>Output:</b>
        ///     If necessary, it recalculates the ctx.Agent’s path based on the ctx.Environment, including updated heatmap data and dynamic conditions.
        ///   </description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx">The current navigation context containing agent and environment data.</param>
        internal override void FindPath(NavigationContext ctx)
        {
            if (ctx.Agent.Path.Count == 0 &&
                ctx.Agent.Destination != Vector2.Zero &&
                (ctx.Agent.Center - ctx.Agent.Destination).Length() >
                ctx.Environment.CellSize.Length() * 0.5f)
                CalculatePath(ctx.Agent, ctx.Environment);
            else if (ctx.Agent.Destination != Vector2.Zero && ctx.Agent.Path.Count > 0)
            {
                _counter++;

                if (_counter < 10)
                    return;

                var currentCell = ctx.Environment.GetCell(ctx.Agent.Center);
                var nextCell = ctx.Environment.GetCell(ctx.Agent.Path.Peek());

                var topLeftCorner = ctx.Agent.Position;
                var topRightCorner = topLeftCorner;
                topRightCorner.X += ctx.Agent.Dimension.X;

                var bottomRightCorner = topLeftCorner + ctx.Agent.Dimension;
                var bottomLeftCorner = bottomRightCorner;
                bottomLeftCorner.X -= ctx.Agent.Dimension.X;

                var cornerCells = new[]
                {
                    ctx.Environment.GetCell(topLeftCorner),
                    ctx.Environment.GetCell(topRightCorner),
                    ctx.Environment.GetCell(bottomRightCorner),
                    ctx.Environment.GetCell(bottomLeftCorner)
                };

                Dictionary<(int X, int Y), uint> agentCorners = [];

                uint delta = 0;
                uint deltaNext = 0;

                foreach (var cell in cornerCells)
                {
                    if (cell == currentCell)
                        delta += ctx.Agent.CellWeight / 4;
                    if (cell == nextCell)
                        deltaNext += ctx.Agent.CellWeight / 4;

                    agentCorners[cell] = agentCorners.TryGetValue(cell, out var existing)
                        ? existing + ctx.Agent.CellWeight / 4
                        : ctx.Agent.CellWeight / 4;
                }

                if (ctx.Environment.Grid.TryGetValue(currentCell, out var current) &&
                    (current - delta > 3 ||
                    (currentCell != nextCell && ctx.Environment.Grid.TryGetValue(nextCell, out var next) &&
                    next - deltaNext > 3)))
                {
                    CalculatePath(ctx.Agent, ctx.Environment, agentCorners, true);
                    _counter = 0;
                }
            }
        }

        /// <summary>
        /// Executes the A* algorithm between the ctx.Agent's current and destination cell.
        /// Optionally resets the graph with updated heatmap weights.
        /// <list type="bullet">
        ///   <item><description><b>Inputs:</b>
        ///     <list type="bullet">
        ///       <item><description><b>ctx.Agent:</b> The moving ctx.Agent for which the path is being calculated. Provides the current position and destination for pathfinding.</description></item>
        ///       <item><description><b>ctx.Environment:</b> The navigable grid representing the ctx.Environment, which contains the cells and their weights, used for pathfinding.</description></item>
        ///       <item><description><b>heatmap:</b> A boolean indicating whether the graph should be recalculated due to updated heatmap values. Default is false.</description></item>
        ///     </list>
        ///   </description></item>
        ///   <item><description><b>Logic:</b>
        ///     Resets the graph if necessary (based on the heatmap flctx.Ag), performs the A* algorithm to calculate the path from the current to the destination cell, 
        ///     and converts the resulting path from grid cells to world positions for the ctx.Agent's movement.</description></item>
        ///   <item><description><b>Output:</b>
        ///     The ctx.Agent's path is updated with the resulting waypoints, which are smoothed using a Bezier curve.</description></item>
        /// </list>
        /// </summary>
        /// <param name="ctx.Agent">The moving ctx.Agent.</param>
        /// <param name="ctx.Environment">The navigable ctx.Environment.</param>
        /// <param name="ctx.AgentCorners">The weight of the ctx.Agent corners.</param>
        /// <param name="heatmap">True if recalculating due to heatmap values.</param>
        private void CalculatePath(
            MovableBody agent,
            NavigableGrid environment,
            Dictionary<(int X, int Y), uint>? agentCorners = null,
            bool heatmap = false)
        {
            var currentCell = environment.GetCell(agent.Center);
            var destCell = environment.GetCell(agent.Destination);

            if (!environment.Grid.ContainsKey(currentCell) ||
                !environment.Grid.ContainsKey(destCell))
                return;

            if (heatmap && agentCorners != null)
                _graph.ClearAll(environment.Grid, agentCorners);
            else
                _graph.ClearAll();

            if (_graph.AStar(destCell, currentCell, out var output) && output != null && output.Steps > 0)
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
        /// Returns an identifier for the A* heatmap pathfinding algorithm.
        /// This method can be used for debugging or logging purposes to identify the algorithm in use.
        /// <list type="bullet">
        ///   <item><description><b>Output:</b> The string "AStarHeatmap", representing the unique identifier for the algorithm.</description></item>
        /// </list>
        /// </summary>
        internal override string GetID()
        {
            return "AStarHeatmap";
        }
    }
}
