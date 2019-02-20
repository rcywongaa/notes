In general, we follow Holzmann's [Power of 10](http://www.spinroot.com/gerard/pdf/P10.pdf) as guidelines. Language specific guidelines are given below:
- [C++](cpp-style-guide.md). Use clang-format config file [here](.clang-format)
- [Python](python-style-guide.md)
- [Java](java-style-guide.md)
- [C#](csharp-style-guide.md)


## Tools
### Build system
- [catkin-lint](https://github.com/fkie/catkin_lint)

### C,C++
- [cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint)
- [clang-format](http://clang.llvm.org/docs/ClangFormat.html)

There may be portions of code that you dont need auto formatting e.g. matrice representation. To disable autoformat in this section of code when using clang-format, do the following
```
int formatted_code;
// clang-format off
    void    unformatted_code  ;
// clang-format on
void formatted_code_again;
```

### Python
- [pylint](http://www.pylint.org/)
- [yapf-format](https://github.com/google/yapf)

To disable autoformat when using yapf, do the following
```
# yapf: disable
FOO = {
    # ... some very large, complex data literal.
}

BAR = [
    # ... another large data literal.
]
# yapf: enable
```
- [prospector](https://prospector.readthedocs.org/en/master/)
