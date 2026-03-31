# Will implement first blueprint level API as first pass and then module level API later. Because additional changes to core needed to handle LCM init for example if im just running `bla  = dimos.connect("UnitreeGo2Connection")` unless we just make sure the agent inits LCM via the command line itself?

So proposed initial pass:

# Simple Example
```python
import dimos

# Normal start and connect
unitree_app = dimos()
> Returns static PER APP UUID inline for agent to save in memory

# Run from all_blueprints.py
unitree_app.run("unitree-g2-basic-blueprint")

# Run as a blueprint object
unitree_app.run(unitree_blueprint)

# Add a additional module that autoconnects dynamically, Returns failed topics that cant find connections
unitree_app.run(yolo_detector_module)

print(app.get_skills())
unitree_app.run_skill("observe")()
> returns string observation

unitree_app.run_skill("relative_move")(distance=1m)
```

# Add another module to the app from above after the fact
```python

# Connect to existing app from above

# Agent forgot what the UUID was lets check
dimos ls
> Returns latest UUID
unitree_app_2 = dimos(uuid)
unitree_app_2.run("openclaw-agent")
```

# Creating a module inline / import new module
```python

# Option 1: write a module inline
class Module1(Module):
 ...

# Option 2: import
import dimos
import yolo_detection_module

# NEW dimos application
drone_app = dimos()

app.run(yolo_detection_module)

app.remapping(/video, /color_image)

app.stop()
```

# Other utilities (pretend they are python)

```python

dimos ls
> App ID | blueprint name | time_started | time running | active/inactive
> xxxx         | bla1
> xxxx

dimos.killall # forced
dimos.stopall # graceful

# App operations
dimos <app-id> list
> Module ID | blueprint name | time_started | time running | active/inactive
> xxxx         | bla1
> xxxx
dimos <app-id> log tail
dimos <app-id> log head <lines>
dimos <app-id> stop
dimos <app-id> kill
dimos <app-id> restart

# Module operations
dimos <app-id> <module-id> log tail
dimos <app-id> <module-id> log head <lines>
dimos <app-id> <module-id> stop
dimos <app-id> <module-id> kill
dimos <app-id> <module-id> restart

# Remapping
dimos <app-id> remap.('/video', '/color_image')





## Dimensional Applications

```
Application (top level — live, long-running instance)
  └── Blueprint (predefined module groups / "run files")
    └── Module (pub/sub primitive)
```

### Key properties of Applications:

1. **Composition**: An Application can compose multiple blueprints + individual modules. e.g. one blueprint + five standalone modules, or two blueprints together.

2. **Long-running**: Applications persist and you can hot-add modules to a running application as you debug/iterate.

4. **Transport & Remapping**: Applications should be able to do transport mapping and remappings, similar to how blueprints do it today. (Remapping at application level is a future pass.)

5. **Relationship to Blueprints**: Blueprints are templates / "run files" that people define. Applications are live instances that consume blueprints. As you construct applications, they can functionally replace blueprints in many cases, but blueprints remain the portable, shareable unit.

6. **Modules as primitives**: Modules remain the core pub/sub primitive. Applications orchestrate them.

### How this maps to the Python API:

```python
import dimos

# Create an application
app = dimos()

# Run a blueprint (adds all its modules to the app)
app.run("unitree-go2-basic")

# Hot-add a standalone module
app.run(yolo_detector_module)

# Compose multiple blueprints
app.run("unitree-go2-basic")
app.run("openclaw-agent")

# Skills come from all modules in the app
app.get_skills()
app.run_skill("observe")()

app.stop()
```
