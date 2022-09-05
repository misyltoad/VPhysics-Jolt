# Building Volt

###### Note: This repository can be built with the default SDK but will not be outlined in this guide.
###### Note: x64 platform builds will also not be outlined in this guide.

All build instructions start the same. Clone a local copy of [mini-source-sdk](https://github.com/Joshua-Ashton/mini-source-sdk). (A minified version of [Source SDK](https://github.com/ValveSoftware/source-sdk-2013) with support for VS 2022)
```bash
git clone https://github.com/Joshua-Ashton/mini-source-sdk.git
```

Following your clone of the mini-source-sdk, navigate to the "version" you would like to build (`sdk2013-mp`, `sdk2013-sp`, `asw`). As an example, the multiplayer version will be used from now on.
```bash
cd mini-source-sdk/sdk2013-mp/src/
```

At this time you should now recursively git clone the VPhysics-Jolt repository into your "src" directory.
```bash
git clone https://github.com/Joshua-Ashton/VPhysics-Jolt.git vphysics_jolt --recursive
```

<strong>From this point onward, please follow the rest of the guide specific to your chosen build platform. The current working directory is also assumed to be `mini-source-sdk/sdk2013-mp/src/` and should be adjusted to suit your own build if need be.</strong>

## Windows

### CLI

Continuing on windows, the next step is to the registry fix script (as administrator). This can be done by right-clicking the `fix_registry.bat` file located in `mini-source-sdk/sdk2013-mp/src/`. If your command prompt or powershell instance are already running under administrator just run the following:
```bash
.\fix_registry.bat
```

This final command to run before the CLI portion of this guide ends. It will generate your [Visual Studio](https://visualstudio.microsoft.com/vs/) project/solution file(s). (You may also just double-click the file `createjoltprojects.bat`)
```bash
.\createjoltprojects.bat
```

### Visual Studio

After running `createjoltprojects.bat` you should now have a file named `jolt.sln`. Open it. After opening building may be as simple as pressing the shortcut `Ctrl+Shift+B` or right-clicking `Solution 'jolt'` bar in the Solution explorer window and pressing `Build Solution`

Congratulations!

The built `vphysics_jolt.dll` file should now be located in `mini-source-sdk/sdk2013-mp/game/bin/`.

### Build errors

Common build errors you may run into building on windows and their solutions.

> #### memoverride.cpp

If there is an error about involving a symbol named ` size_t _msize` or `size_t __cdecl _msize_base` try removing or commenting out definitions of <strong>both</strong>. You should leave only the code below:
```cpp
size_t msize( void *pMem )
{
	return g_pMemAlloc->GetSize(pMem);
}
```

Where the code to be removed/commented would be:
```cpp
size_t __cdecl _msize_base( void *pMem ) noexcept
{
	return g_pMemAlloc->GetSize(pMem);
}

size_t _msize( void *pMem )
{
	return _msize_base(pMem);
}
```

###### It may be wise to try different variations if the given solution does not work.

## Linux

Continuing on linux, there are a few prerequisites to building regardless of distrobution.
- glibc v2.29+
- gcc v8+
- g++ v8+
- gcc-multilib
- g++-multilib
- make

###### Note: Attempts to compile with older versions of glibc are undocumented and probably will not work (you are still welcome to confirm this)

### Debian/Ubuntu
###### Note: Some older versions of debian or debian-based distrobutions (i.e. ubuntu) may use `apt-get`
###### Note: Older versions may also require version explicit packages if available.

Before we continue building project files, lets get our dependencies out of the way.
Install the `build-essential` group to get common developer packages such as gcc, g++, and make, which are included.
```bash
sudo apt install build-essential
```

After installing the build-essential group packages, we now need to install the x86 dependencies.
```bash
sudo apt install gcc-multilib g++-multilib
```

The next step is to build our makefiles by running the premade script `createjoltprojects.sh`
```bash
./createjoltprojects.sh
```

Once the script has finished running you should now have a file name `jolt.mak` in the current directory. Verify you do and now run it with the command below. This command automatically will (and is recommended to) use all available CPU threads. If you want to specify it yourself, replace `` `nproc` `` with a number less than or equal to your max number of threads. To be clear, this only affects speed.
```bash
make -j `nproc` -f jolt.mak
```

Alternative example...
```bash
make -j 8 -f jolt.mak
```

Congratulations!

If the console output `LINKING` AND `COPYING TO` messages, that means your build was successful and you may retrieve your `vphysics_jolt.so` and `vphysics_jolt_srv.so` files which will be located in `mini-source-sdk/sdk2013-mp/game/bin/`.

### Build errors (distrobution independent)

Common build errors you may run into building on linux and their solutions.

> #### error: unrecognized command line option ‘-std=gnu++20’; did you mean ‘-std=gnu++2a’? (make: *** [jolt.mak:28: all] Error 2)

This error may come up between different versions of g++, the easist way to fix this is to edit `devtools/makefile_base_posix.mak` at approximately `line 47` and change `-std=gnu++20` to `-std=gnu++2a`.

The full change should like similar to this. Starting as:
```makefile
CXXFLAGS = $(CFLAGS) $(WARN_CXX_FLAGS) -std=gnu++20 -Wno-narrowing -Wno-register -Wno-deprecated-enum-enum-c    onversion -Wno-deprecated-declarations -fpermissive -Wno-volatile -Wno-ignored-attributes -I/usr/include/freetype2
```
Changing to:
```makefile
CXXFLAGS = $(CFLAGS) $(WARN_CXX_FLAGS) -std=gnu++2a -Wno-narrowing -Wno-register -Wno-deprecated-enum-enum-c    onversion -Wno-deprecated-declarations -fpermissive -Wno-volatile -Wno-ignored-attributes -I/usr/include/freetype2
```
<br>

> #### Other obscure compile error

Try upgrading your gcc/g++ toolset to a higher version if possible.

## OSX
Unsupported.

¯\\\_(ツ)\_/¯
