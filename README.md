# Arduino Development Kit 🇮🇹 ☕

Template repository for developing Eurorack modules using Arduino/ATMEGA and PlatformIO

## Installation ⛏️

Make sure platform io command line tools are installed

```
brew install platformio
```

In the `messd-up` project directory, install the external libraries

```
pio lib install
```

## Development 🚠

After creating a feature branch and making your changes, you can build locally without the board plugged in to make sure the code can compile. There are two building environments specified in `platform.ini`:

-   `[env:uno]` is the default environment, for uploading and onboard integration testing
-   `[env:native]` is for building, uploading, and unit testing

To perform unit testing and building without the board plugged in you can simply run

```
pio test -e native && pio run
```

Then plug the prototype board into your computer to perform integration tests

```
pio test -e uno
```

Finally, upload the code with the optional `monitor` flag to view the output of any Serial printing put in the code for debugging.

```
pio run --target upload --target monitor
```

After pull requesting, Travis CI will make sure that the code is able to compile and perform integration testing in order to make sure everything is squeaky clean before merging

If you're using clangd as a build tool, you can generate compile compile commands for linting

```
pio run -t compiledb
```
