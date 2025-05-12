# UVMapper — Least Squares Conformal Mapping (LSCM)

This project implements UV unwrapping using Least Squares Conformal Mapping (LSCM) to generate conformal (angle-preserving) UV maps for 3D meshes.

## Dependencies

Install required system packages:

```bash
sudo apt install build-essential libglfw3-dev libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev libeigen3-dev
```

## Build Instructions

To compile the project, run:

```bash
./build.sh
```

This builds the `UVMapper` executable in the root directory.

## Usage

```bash
./UVMapper <filename>
```

* Input files should be placed in the `resources/` directory.
* The resulting UV-unwrapped mesh will be saved as `out.obj` in the `resources/` directory.

## Example

```bash
./UVMapper resources/icosahedron.obj
```

This processes `resources/icosahedron.obj` and writes the output mesh with UVs to `out.obj`.

## Notes

* Only OBJ-format meshes are supported.
* Input meshes must be triangulated.
