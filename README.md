# Industry 4.0 Simulation Framework

This project simulates a modular, factory environment inspired by Industry 4.0 principles. It includes autonomous transport units, configurable production stations and control via UI. The simulation emphasizes configurability, traceability and runtime introspection. ([Demo](https://youtu.be/Y9loZ_bbvng))

## 📦 Features

- Interactive blueprint loader with pauseable runtime
- Actor-based logic using [Akka.NET](https://getakka.net/)
- Autonomous product flow simulation
- Modular production and transport logic
- Detailed runtime logging and heatmap generation
- Visualization with Win32 + Direct2D

---

## 💡 Use Cases

- Testing Industry 4.0 strategies
- Research in multi-agent systems
- Evaluating pathfinding and steering behaviors
- Simulating decentralized product negotiation and transport

---

## 🚀 Getting Started

1. Clone this repository.
2. Open in Visual Studio 2022 or newer.
3. Set the `Simulation(WinExe)` project as the startup project.
4. Build and run the solution.

> ✅ Make sure blueprints are placed in the `Blueprint/` folder inside the executable directory.

---

## 🧭 User Interface Overview

### 🔵 Load Screen
![image](https://github.com/user-attachments/assets/097ea31b-4a99-48e0-8ad4-5a108db67858)

- On startup, a load screen displays available simulation blueprints (JSON format).
- Navigate via `< Prev` and `Next >`, and click a blueprint name to start.
![image](https://github.com/user-attachments/assets/1c00e7cc-21c7-4a9c-abb3-8b47efc2be2c)

### 🟢 Simulation View
![image](https://github.com/user-attachments/assets/094f8cd4-516c-4c24-b5a5-b1659cd766d1)

- A paused simulation loads with visualized grid, producers, movers, and forbidden zones.
- Real-world dimensions (in mm) make movement speeds interpretable.

### ⚙️ Settings Panel
![image](https://github.com/user-attachments/assets/3a3ef189-623b-4624-8625-98f8b089cbc2)

- Open with the settings icon or `Ctrl`.
- Toggle features like:
  - Pause/resume (`Spacebar`)
  - Enable overlays: pathfinding, steering vectors, borders, heatmap, etc.
  - Adjust UPS/FPS sliders
  - Enable/disable detailed logging
  - Return to blueprint selection

![image](https://github.com/user-attachments/assets/5513ba89-ef84-4a3f-ac3f-a72f6bdb6e85)

> When exiting(`Escape`), a dump file is automatically created in the `Output/` folder.

---

## 🖱️ Unit Interaction

![image](https://github.com/user-attachments/assets/75b2677f-dad7-431e-8cc1-722609eeacd9)
- **Right-click**: Toggle unit status between `Alive` and `Blocked`.

![image](https://github.com/user-attachments/assets/9dd9edaf-d12c-4624-b782-245c07cda863)
- **Left-click**: Inspect real-time telemetry of producers and movers.

Overlays show:
- Position, velocity, acceleration
- Assigned tasks and product information
- Live processing status and tick counters

---

## 🧪 Configurations

Simulation behavior is defined by:
- **Blueprints** (`Blueprint/*.json`)
- **Code-based setup** (e.g., `Setup()`, `SetupCost()` methods)

Modular components allow you to:
- Create new units, product types, or behaviors
- Define transport costs and steering strategies
- Adjust production rules via `UnitProduction` config

---

## 🧱 Architecture Overview

The simulation is built using a modular, layered architecture to support extensibility and runtime configuration.

### 🏗️ Project Structure

- **Simulation** (`Simulation/`): Core logic, user interface, Akka.NET actor system, runtime orchestration.
- **Win32** (`Win32/`): Native rendering and window lifecycle handling using P/Invoke and Vortice.Direct2D1.

### 📦 Key Components

| Component       | Responsibility |
|----------------|----------------|
| `Windower`     | Bridges native Win32 events (keyboard, mouse, window) with the simulation layer. |
| `Renderer`     | Facade for Direct2D rendering. Handles drawing units, overlays, debug layers. |
| `Environment`  | Shared simulation state, including agents (Movers, Producers), navigation grid, zones. |
| `Procedure`    | External communication logic: either dummy product generation or live MQTT integration. |
| `Cycle`        | Manages simulation timing, tick updates, and rendering intervals. |
| `ActorSystem`  | Akka.NET-based concurrency model, enabling decoupled, event-driven simulation. |
| `Dumper`       | Logs runtime data for benchmarking and traceability. |

### 🔄 Runtime Flow

1. **Initialization**: Load blueprint (JSON) and configure the simulation environment.
2. **Main Loop**: Update and render cycles run on separate threads (UPS/FPS adjustable).
3. **Interaction**: Keyboard/mouse events control simulation behavior and UI toggles.
4. **Logging**: At the end of a run, dump files are created for analysis.

### 💡 Design Principles

- **Modular**: Behavior, navigation and cost models can be replaced independently.
- **Extensible**: New units, algorithms, or communication protocols can be added without rewriting core logic.
- **Native Performance**: Rendering is handled via low-level Direct2D bindings for speed and control.
- **Actor-Based**: Akka.NET actors ensure concurrency and robustness in unit coordination.

## 🧾 Logging & Analysis

- All unit and product activity is logged
- Heatmaps and performance metrics are auto-generated
- Output folder includes:
  - Full tick-based logs
  - Dump files
  - Metadata with blueprint IDs and run configurations

---
