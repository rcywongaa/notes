In general, we follow Google C++ Coding [style](http://google-styleguide.googlecode.com/svn/trunk/cppguide.html)

We use the following exceptions:
- gstyle states that ```Each line of text in your code should be at most 80 characters long```. We will use 100 lines instead for easy readability
- gstyle states that ```input arguments are values or const references while output arguments are pointers``` (http://google-styleguide.googlecode.com/svn/trunk/cppguide.html#Reference_Arguments). We do not strictly enforce this convention and even encourage the use of references instead of pointers for output arguments in some parts of he codebase. Explanation : While it is true that systematically using pointers for output arguments allows to identify easily, at the call site, whether the argument is an input or an output argument, the use of pointers comes with the usual risk of dereferencing a null pointer. When the null value is not used by the function itself, we recommend using references for library functions that can be potentially used by less experienced developers.

## Configuring Eclipse for auto-formatting with the Google C++ style

Download the Google cpp format file for eclipse at https://github.com/google/styleguide/blob/gh-pages/eclipse-cpp-google-style.xml

In Eclipse, go to Windows>Preferences>C/C++>Code Style>Formatter>Import and select your file

To set Eclipse to auto-format your files upon save (recommended), go to Windows>Preferences>C/C++>Editor/Save Actions and tick "Format source code"
