## Follow these instructions to open Lab 2 project and add our main.c file 
Only the file main.c has been modified from the original Lab 2 project.

1. Launch STM32CubeIDE.
2. A STM32 Project window will open, find the board in the board selector.
3.  On the file tab, select 'Open Project from File System'. 
4.  Find Lab 2 project location and select it. 
5.  Import project. 
6.   Select 'Copy projects into workspace' under Options.
8. Click finish on the bottom right. 
9. On the project explorer, expand 'Core' and then 'Src'. 
10. Right click, select 'Add file'. 
11. Find the 'main.c'location and select it. 
12. Click finish. 
13. Debug the program. 

## Follow these instructions to replicate our results

1. Comment out line 35 to disable block processing and collect the original results.
2. Uncomment line 34 to run the code without timer interrupts.
3. Set a breakpoint at line 279 to stop the program once filteredOutputBufferA has been filled for the first time. 
4. In the Expressions tab, set a watch statement for filteredOutputBufferA.
5. Debug and run the program, then record the contents of filteredOutputBufferA.
6. Uncomment line 35 to enable block processing, and repeat steps 2-5 for each of the new functions.
7. To run ProcessSample(), comment out line 38 and leave line 39 uncommented.
8. To run ProcessSample2(), comment out line 38 and line 39.
9. To run ProcessSample3(),leave line 39 and line 38 uncommented.
10. To run ProcessSample4(), comment out line 39 and leave line 39 uncommented.