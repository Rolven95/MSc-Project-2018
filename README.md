# MSc Project Repo

This is the shared project repository for both the Sandbox API and the Urban Planner projects.
Both projects use cmake to generate Visual Studio solution files.
Therefore you should *not* be creating your own visual studio files!
Simply run `python build.py` from the root directory of the desired project and it will create a `bin` folder.
You can then simply fire up the visual studio file that cmake created and work from there.

When adding new source files to the project, remember to reference them in the `CMakeLists.txt` file as the Visual Studio solution file is generated everytime, so it won't remember where your source files are!