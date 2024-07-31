# Cognitive Heuristics Steering Agent Implementation for the SteerSuite

## Installations

This project depends on steersuite, which can be extracted or symlinked to
`external/steersuite`.

To install the project, run the following commands:

```bash
cd build
cmake ..
make
```

## Usage

### AI only

In steersuite, set the ModuleLoadPath to `build/` and add parameter `-ai cog-ai`.

### AI with Scenarios

Link project to `build/cog-ai` and compile scenario module.
