# Conventions
To make it easier for the members of the team to understand each other's contributions to this repo, and to create a cohesive codebase, we have put in place the following conventions which are expected to be followed when developing on this repo

# Contents
- [Packages](packages)
- [Programming](programming)
    - [Python Conventions](python-conventions)
    - [C++ Conventions](c++-conventions)

# Packages
When creating a new package, or 
# Programming

## Python Conventions

For the python programming conventions we use, they loosely follow the [PEP](https://www.python.org/dev/peps/pep-0008/) guidelines. The following list gives the conventions that should be followed

- Module names are **under_scored**
- Class names are **CamelCase**
- Function names are **under_scored**
- Functions that are "helper" or "private" should be prefixed by **_**
- All "helper" or "private" functions should be put below regular functions
- Arguments and variables are **under_scored**
- Leave **2** lines between functions and classes
- Imports should be organized **Standard, Third-Party, Application**
- Functions should be documented as per [**Epytext**](http://epydoc.sourceforge.net/epytext.html)

Here is an example python module displaying these conventions
```
# example_module.py
import some_standard_library
import another_standard_library

from some_third_party_library import ExtendedClass

import some_application_library
import another_application_library


class ExampleClass(ExtendedClass):
    """
    Optional class description
    """


    def __init__(self, arg_one, arg_two):
        """
        Short Explanation

        Longer explanation where you describe more in depth
        the purpose that this function has

        @param arg_one: A description of arg_one
        @param arg_two: A description of arg_two

        """
        self._helper_function(arg_one)
        self.member_var = arg_two


    def member_function(self, arg_one):
        """
        Short Explanation

        Longer explanation where you describe more in depth
        the purpose that this function has

        @param arg_one: A description of arg_one
        @type: An optional description of arg_one's type
        @return: Description of return value

        """
        local_var = 1
        return some_application_library.special_function(arg_one, local_var)

    
    def _helper_function(self, arg_one):
        """
        Short Explanation

        Longer explanation where you describe more in depth
        the purpose that this function has

        @param arg_one: A description of arg_one
        @type: An optional description of arg_one's type
        @return: Description of return value
        """
        arg_one += 5
        return arg_one


class AnotherExampleClass:


    def __init__(self, arg_one):
        """
        Short Explanation

        Longer explanation where you describe more in depth
        the purpose that this function has

        @param arg_one: A description of arg_one
        """
        self.member_var = arg_one


"""
Description of main program
"""
def main():
    example_class_obj = ExampleClass(1, 2)


````

## C++ Conventions