# Information for Developers

## Coding Conventions

* Classes are Uppercase, methods and functions lowerMixedCase.
* Apart from those naming conventions, we adopt Google C++ style.
* Use meaningful variable names, e.g. `measurement` not `msm`, avoid abbreviations.

### Header-Wrapper Parameter Name Matching

If you add a C++ function to a `.i` file to expose it to the wrapper, you must ensure that the parameter names match exactly between the declaration in the header file and the declaration in the `.i`. Similarly, if you change any parameter names in a wrapped function in a header file, or change any parameter names in a `.i` file, you must change the corresponding function in the other file to reflect those changes. 

This includes inherited functions wrapped in `.i` files. If `Pose2` inherits `logmap` from `LieGroup` (in `Lie.h`) and wraps `logmap`, `Pose2.logmap` in the `.i` needs to have the same parameter names as the function definition in `Lie.h`. 

> [!IMPORTANT]
> The Doxygen documentation from the C++ will not carry over into the Python docstring if the parameter names do not match exactly!

If you encounter any functions that do not meet this criterion, please submit a PR to make them match.

## Windows

On Windows it is necessary to explicitly export all functions from the library which should be externally accessible. To do this, include the macro `GTSAM_EXPORT` in your class or function definition.

For example:
```cpp
class GTSAM_EXPORT MyClass { ... };

GTSAM_EXPORT return_type myFunction();
```

More details [here](Using-GTSAM-EXPORT.md).
