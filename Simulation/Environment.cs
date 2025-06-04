using Akka.Event;
using Blueprint = Simulation.Scene.Blueprint;
using Borders = Simulation.Scene.Borders;
using CloseCommonWriter = Simulation.Util.CloseCommonWriter;
using Color = Simulation.Util.Color;
using ForbiddenZones = Simulation.Scene.ForbiddenZones;
using Movers = Simulation.Scene.Movers;
using NavigableGrid = Simulation.Scene.NavigableGrid;
using ParkingSpaces = Simulation.Scene.ParkingSpaces;
using Procedure = Simulation.Dummy.Procedure;
using Producers = Simulation.Scene.Producers;
using Vector2 = System.Numerics.Vector2;

namespace Simulation
{
    /// <summary>
    /// Represents the simulation environment, managing movers, producers, forbidden zones, parking areas,
    /// rendering logic and spatial grid data.
    /// 
    /// Implements a singleton pattern to provide centralized access to environment state and services.
    /// </summary>
    internal sealed class Environment : NavigableGrid
    {
        /// <summary>
        /// Identifier for the current simulation environment instance, typically derived from the blueprint name.
        /// </summary>
        internal string ID { get; private set; } = null!;

        /// <summary>
        /// Manages all transport units ("movers") in the environment, including updates, spatial lookup and collision detection.
        /// </summary>
        internal Movers Movers { get; private set; } = null!;

        /// <summary>
        /// Represents the spatial boundaries of the simulation grid,
        /// used for detecting and handling collisions with world edges.
        /// </summary>
        internal Borders Borders { get; private set; } = null!;

        /// <summary>
        /// Manages all production units ("producers") in the environment, including placement, processing state and rendering.
        /// </summary>
        internal Producers Producers { get; private set; } = null!;

        /// <summary>
        /// Handles assignment, tracking and release of parking spaces used by idle movers.
        /// </summary>
        internal ParkingSpaces Parkings { get; private set; } = null!;

        /// <summary>
        /// Defines restricted zones where movement is not allowed.
        /// Used for navigation and spatial grid generation.
        /// </summary>
        internal ForbiddenZones ForbiddenZones { get; private set; } = null!;

        /// <summary>
        /// Tracks the total number of collision events detected in the environment during the simulation.
        /// </summary>
        internal uint Collisions { get; set; }

        /// <summary>
        /// Full path to the output directory where logs, dumps and other simulation data are written.
        /// </summary>
        internal string OutputDir { get; private set; } = null!;

        /// <summary>
        /// Cached scaled size of a single grid cell for rendering.
        /// </summary>
        internal Vector2 RenderCell { get; set; }

        /// <summary>
        /// Set of cell coordinates currently marked for heatmap visualization.
        /// </summary>
        private readonly HashSet<string> _heatCells = [];

        private static Environment? _instance;
        private static readonly object _lock = new();

        /// <summary>
        /// Singleton instance of the <see cref="Environment"/> class.
        /// Throws an exception if accessed before <see cref="Initialize"/> is called.
        /// </summary>
        internal static Environment Instance
        {
            get
            {
                if (_instance == null)
                    throw new InvalidOperationException("Environment is not initialized, call the Environment.Initialize() function.");

                return _instance;
            }
        }

        /// <summary>
        /// Initializes the singleton environment instance from a provided blueprint.
        /// Sets up all core components including movers, producers, spatial grid and output directories.
        /// </summary>
        internal static void Initialize()
        {
            lock (_lock)
                _instance ??= new Environment();
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>Environment</b>: Initializes the simulation environment in load screen mode.</description></item>
        /// <item><description>Sets default values for ID, dimensions, cell size, scale, and offset.</description></item>
        /// <item><description>Configures the environment as a splash screen where users can later select a blueprint to run.</description></item>
        /// <item><description>Initializes all core systems: forbidden zones, producers, grid, borders, movers, and parking spaces.</description></item>
        /// <item><description>Prepares navigation and assigns default cell weights for movers.</description></item>
        /// <item><description>Creates a unique output directory based on timestamp and GUID for storing simulation data.</description></item>
        /// </list>
        /// </summary>
        private Environment()
        {
            LoadScreen();
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>LoadScreen</b>: Configures the environment as the initial load/splash screen for blueprint selection.</description></item>
        /// <item><description>Resets ID, cell size, scale, and offset to defaults.</description></item>
        /// <item><description>Sets screen dimensions and resets collision count.</description></item>
        /// <item><description>Initializes empty forbidden zones, producers, grid, borders, movers, and parking spaces.</description></item>
        /// <item><description>Precomputes cell weights and navigation for movers.</description></item>
        /// <item><description>Generates a unique output directory based on timestamp and GUID.</description></item>
        /// </list>
        /// </summary>
        internal void LoadScreen()
        {
            ID = "LoadScreen";

            CellSize = Vector2.Zero;
            Dimension = new Vector2(1920, 1080);
            Collisions = 0;

            ForbiddenZones = new ForbiddenZones([], CellSize);

            Producers = new Producers(
                [],
                [],
                ForbiddenZones,
                Vector2.Zero,
                CellSize,
                0
                );

            Grid.Clear();

            Borders = new Borders([], CellSize);

            Movers = new Movers([], [], CellSize);

            Parkings = new ParkingSpaces([]);

            SetCellWeights([]);
            Movers.InitNavigation([]);

            var timestamp = DateTime.Now.ToString("yyyyMMddHHmmss");
            var guid = Guid.NewGuid().ToString("N")[..16];
            var name = $"{ID}_{timestamp}_{guid}";

            var outDir = Path.Combine(AppContext.BaseDirectory, $"Output/{name}");
            OutputDir = outDir;
            Renderer.Instance.WorldDimension = Dimension;
        }

        /// <summary>
        /// <list type="bullet">
        /// <item><description><b>LoadBlueprint</b>: Initializes the full environment using the selected blueprint from the splash screen.</description></item>
        /// <item><description>Loads blueprint data from file and applies configuration (tick cap, cell size, dimensions, etc.).</description></item>
        /// <item><description>Initializes all simulation components: forbidden zones, producers, grid, borders, movers, and parking spaces.</description></item>
        /// <item><description>Builds navigation data and assigns movement costs for all movers.</description></item>
        /// <item><description>Generates a unique output directory path for logging and data export.</description></item>
        /// <item><description>Starts the simulation cycle and initializes MQTT communication and procedural logic.</description></item>
        /// <item><description>Clears the display, updates the viewport, and triggers grid rendering.</description></item>
        /// </list>
        /// </summary>
        internal void LoadBlueprint(string file)
        {
            var blueprint = Blueprint.LoadFromFile(file);
            Cycle.TickCap = blueprint.TickCap;

            ID = blueprint.Name;

            CellSize = blueprint.CellSize;

            Dimension = blueprint.Dimension;
            Renderer.Instance.WorldDimension = Dimension;
            Collisions = 0;

            ForbiddenZones = new ForbiddenZones(blueprint.ForbiddenZones, CellSize);

            Producers = new Producers(
                blueprint.Producers,
                blueprint.ProducerGroups,
                ForbiddenZones,
                new Vector2(blueprint.MoverMaxExtent, blueprint.MoverMaxExtent),
                CellSize,
                blueprint.ProducerMaxQueue
                );

            GenerateGrid(ForbiddenZones.GetAllForbiddenZoneCells());

            Borders = new Borders(Grid, CellSize);

            Movers = new Movers(blueprint.Movers, blueprint.MoverGroups, CellSize);

            Parkings = new ParkingSpaces(Movers.Get());

            SetCellWeights(Movers.GetAllCellWeights(this));
            Movers.InitNavigation(Grid);

            var nav = Movers.Get().First().Navigation.GetID();
            var mov = Movers.Get().First().Cost.GetID();
            var prod = Producers.Get().First().Cost.GetID();
            var id = $"{ID}_{nav}_{mov}_{prod}";

            var timestamp = DateTime.Now.ToString("yyyyMMddHHmmss");
            var guid = Guid.NewGuid().ToString("N")[..16];
            var name = $"{id}_{timestamp}_{guid}";

            var oldDir = OutputDir;
            OutputDir = Path.Combine(AppContext.BaseDirectory, $"Output/{name}");
            App.System.EventStream.Publish(new CloseCommonWriter(oldDir, OutputDir));

            Cycle.Start();
            Procedure.Instance.Setup(Cycle.TargetUPS, (uint)Movers.Get().Count);
            Renderer.Instance.Clear();

            App.Log.Info("[Environment] Initialized with blueprint: {0}", file);
            Renderer.Instance.UpdateViewport((int)Renderer.Instance.ScreenDimension.X, (int)Renderer.Instance.ScreenDimension.Y);
            RenderGrid();
        }

        /// <summary>
        /// Increments the total collision count by the specified number.
        /// </summary>
        /// <param name="collisions">The number of collisions to add.</param>
        internal void AddCollision(uint collisions)
        {
            Collisions += collisions;
        }

        /// <summary>
        /// Updates internal rendering scale, offset and viewport metrics based on the current window size.
        /// Also refreshes UI scaling and invokes unit-level viewport updates and rendering.
        /// </summary>
        /// <param name="scale">The width of the simulation viewport in pixels.</param>
        internal void UpdateViewport(float scale)
        {
            RenderCell = CellSize * scale;

            foreach (var mover in Movers.Get())
                mover.UpdateViewport(scale);

            foreach (var producer in Producers.Get())
                producer.UpdateRendering(scale);

            foreach (var noGo in ForbiddenZones.Get())
                noGo.UpdateRendering(scale);

            RenderScene();
        }

        /// <summary>
        /// Executes the rendering pipeline for the simulation environment,
        /// drawing all movers, producers, forbidden zones and the simulation boundaries.
        /// </summary>
        private void RenderScene()
        {
            RenderBackground();

            foreach (var mover in Movers.Get())
                mover.Render();

            foreach (var producer in Producers.Get())
            {
                producer.Render();
                producer.RenderProcessing();
            }

            foreach (var noGo in ForbiddenZones.Get())
                noGo.Render();

            if (UI.Instance.SettingPanel.Grid.Active)
                RenderGrid();

            if (UI.Instance.SettingPanel.Border.Active)
                RenderBorders();

            UI.Instance.Render();
        }

        /// <summary>
        /// Renders the border's of the environment in the simulation view.
        /// </summary>
        internal void RenderBorders()
        {
            var thickness = 1.0f;

            foreach (var kvp in Borders.Get())
                foreach (var (A, B) in kvp.Value)
                    Renderer.Instance.DrawLine(
                        $"9_{kvp.Key}_{A.X}_{A.Y}_{B.X}_{B.Y}",
                        A * Renderer.Instance.Scale + Renderer.Instance.Offset,
                        B * Renderer.Instance.Scale + Renderer.Instance.Offset,
                        Color.Purple,
                        thickness
                        );
        }

        /// <summary>
        /// Renders the environment grid.
        /// </summary>
        internal void RenderGrid()
        {
            if (Grid.Count <= 0)
                return;

            var id = 0;

            for (
                var x = Renderer.Instance.Offset.X;
                x < Renderer.Instance.ScreenDimension.X - Renderer.Instance.Offset.X - RenderCell.X * 0.5;
                x += RenderCell.X
                )
                for (
                    var y = Renderer.Instance.Offset.Y;
                    y < Renderer.Instance.ScreenDimension.Y - Renderer.Instance.Offset.Y - RenderCell.Y * 0.5;
                    y += RenderCell.Y
                    )
                    Renderer.Instance.DrawRectangle(
                        $"1_{ID}{++id}",
                        new Vector2(x, y),
                        RenderCell,
                        Color.White75,
                        false,
                        0.0f,
                        0.25f
                        );
        }

        /// <summary>
        /// Used for partial scene updates, renders only the units (movers and processing position of producers).
        /// While controlling if the heatmap or debug view has to be rendered.
        /// </summary>
        internal void Render()
        {
            foreach (var mover in Movers.Get())
                mover.Render();

            foreach (var producer in Producers.Get())
                producer.RenderProcessing();
        }

        /// <summary>
        /// Renders a heatmap overlay of agent density in the environment.
        /// </summary>
        internal void RenderHeatmap()
        {
            foreach (var kvp in Grid)
            {
                var heatID = $"2_{ID}{kvp.Key.X},{kvp.Key.Y}";
                var agentCount = Grid.TryGetValue((kvp.Key.X, kvp.Key.Y), out var counts) ? counts : 0;

                if (agentCount > 0)
                {
                    var x = Renderer.Instance.Offset.X + kvp.Key.X * RenderCell.X;
                    var y = Renderer.Instance.Offset.Y + kvp.Key.Y * RenderCell.Y;

                    var color = Color.GrayLight15;

                    if (agentCount > 3 && agentCount < 8)
                        color = Color.Yellow50;
                    else if (agentCount >= 8 && agentCount < 13)
                        color = Color.Trump50;
                    else if (agentCount >= 13)
                        color = Color.Red50;

                    Renderer.Instance.DrawRectangle(heatID, new Vector2(x, y), RenderCell, color);
                    _heatCells.Add(heatID);
                }
                else
                    Renderer.Instance.RemoveDrawCommand(heatID);
            }
        }

        /// <summary>
        /// Renders the background of the environment, used as base color.
        /// </summary>
        internal void RenderBackground()
        {
            var pos = new Vector2(0, 0) + Renderer.Instance.Offset;
            var dim = Dimension * Renderer.Instance.Scale;

            Renderer.Instance.DrawRectangle($"0_{ID}", pos, dim, Color.GrayDark);
        }

        /// <summary>
        /// Removes the environment border from the render view.
        /// </summary>
        internal void RemoveBorders()
        {
            foreach (var kvp in Borders.Get())
                Renderer.Instance.RemoveKeyContainedDrawCommand($"9_{kvp.Key}");
        }

        /// <summary>
        /// Removes the grid overlay from the render view.
        /// </summary>
        internal void RemoveGrid()
        {
            Renderer.Instance.RemoveKeyContainedDrawCommand($"1_{ID}");
        }

        /// <summary>
        /// Removes the heatmap overlay from the render view.
        /// </summary>
        internal void RemoveHeatmap()
        {
            if (_heatCells.Count <= 0)
                return;

            Renderer.Instance.RemoveKeyContainedDrawCommand($"2_{ID}");
            _heatCells.Clear();
        }

        /// <summary>
        /// Removes the background of the environment.
        /// </summary>
        internal void RemoveBackground()
        {
            Renderer.Instance.RemoveDrawCommand($"0_{ID}");
        }
    }
}
