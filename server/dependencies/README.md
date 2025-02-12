# Structure of Dependencies
Dependencies are handled differently depending on whether they have/require their own build system (or whether it is small or large)

`sources` contains smaller dependencies without their own build system, compiled by the main build system

`buildfiles` includes fetch and build scripts to compile dependencies with their own build-systems, with the resulting files in `include`, `lib` 
- the exception is Eigen, which is fetched via a script despite having no build process, as it contains many files. This allows frequent, transparent updates

## Licenses

The build scripts are licensed under the MIT license. <br>
For licenses of the dependencies in `sources` or those fetched/built by the build scripts, see the `licenses` folder in the parent directory.