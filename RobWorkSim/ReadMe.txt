
RWSim is a simulator that 

The project is setup into different folders:
src - files under src folder is compiled into a library
plugin - The plugin folder contain plugins and qt specific libs that will link with the compiled library
test - The test folder contain testsrc that will link with the compiled library
example - The example folder contains example files that link with the compiled library


To include extra libraries or includedirectories to all source add it to CMakeLists.txt
in the root folder. Else add it to the specific CMakeLists.txt folder under the folder 
src, test, example or plugin.

Quick HowTo:

- Copy the complete sampleproject folder and rename it to your name of choice.
- Edit the CMakeFile in your root folder and make sure the variable Root, RWRoot, RWSRoot is
  setup correctly. Also make sure to rename the project name and if needed add extra includedirs 
  and libraries.
- When adding source files to your project make sure that you remember to add them in the CMakeLists.txt
  in the folder where you add the source file
  
