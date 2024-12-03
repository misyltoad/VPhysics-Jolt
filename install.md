# Installing Volt

## 32bit Game

1. Download the latest [release](https://github.com/Joshua-Ashton/VPhysics-Jolt/releases)  
2. Copy the contents of the zip into the game's root directory and replace all files.  

## 64bit Game

1. Download the latest [release](https://github.com/Joshua-Ashton/VPhysics-Jolt/releases)  
2. Copy all files located in the bin folder (from the ZIP) into the bin folder the game uses.  
> NOTE: The path could be `bin/win64` or `bin/linux64`  

## Gmod Dedicated Server

1. Download the latest [release](https://github.com/Joshua-Ashton/VPhysics-Jolt/releases)  
2. Copy all files located in the bin folder (from the ZIP) into the bin folder the game uses.  
> NOTE: The path normally is `bin`, but could be `bin/win64` or `bin/linux64` if you use the `x86-64` Branch.  
3. (**Linux only**) Check if a vphysics_srv.so exists in the bin folder. If it does, remove the `vphysics_srv.so` and rename the `vphysics.so` to `vphysics_srv.so`.  
