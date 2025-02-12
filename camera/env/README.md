# Pi-Specific Environment
This directory includes source code files merely for the purpose of providing the editor LSPs like clangd with a valid environment to ease development. <br>
None of the files in this directory are included in the build as they are part of the piCore system distribution. <br>
`build_commands.sh` is provided so that bear can be used to record the commands and generate a valid `compile_commands.json` file for clangd.