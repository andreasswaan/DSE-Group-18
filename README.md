Some general rules to write to this repository

Use the following variable and function naming conventions:

module_name, package_name, ClassName, method_name,
ExceptionName, function_name, GLOBAL_CONSTANT_NAME,
global_var_name, instance_var_name, function_parameter_name,
local_var_name

ALWAYS PULL and perform ALL the tests before merging !!!

Commit often, and write useful messages, (bad examples: test1, commit, bug fix,...), this will allow the team to track changes and bugs
(If you use vscode you can make use of the extension to automatically generate AI messages)

Make sense of you branches and name them like this: making-drone-classes

Use object oriented programming, with the use of classes, for example every drone could be a different object of class Drone.

If you use VScode please use Black Formatter as an extension: https://marketplace.visualstudio.com/items/?itemName=ms-python.black-formatter

If you use VScode please use Python Indent as an extension: https://marketplace.visualstudio.com/items/?itemName=KevinRose.vsc-python-indent

When writing functions please specify the type of every entry and the type of the returned value, please specify None if nothing gets returned
example:

    def my_function(number: int, string:str, number2:float, ) -> int:
        return number \* number

When writing you code think of the unit test writing afterwards, meaning that ONE function DOES ONE function,
if a function does 2 very different things, rewriting is needed.

Unit testing should include at least 3 tests:

-Type error: for example if you don't include an input to a functions an error should be returned
EX:

    with pytest.raises(TypeError):
        tTAT_to_tSAT(TAT_deg)

-DIV by 0 error: if the function include a division please check if the denominator is 0 you get the div by 0 error
EX:

    with pytest.raises(ZeroDivisionError):
        cl_cd_calc(weight, 0, vTAS, thrust)

-Check if one of the inputs change that the end results also changes:
EX:

    assert correct_value_cl != calculated_value_cl_1, "CL calculation is wrong"

UNIT test should be written AT THE SAME time as the function, this will allow the team to test changes before merging to the branch.

All unit tests that intend to remain in the program should be written using pytest in a separate file.


---------------------------------------------------

# DSE Assignment: Group 18

## VS Code Extensions
- autoDocstring
- Jupyter
- GitLens
- Rainbow CSV
- vscode-pdf


## Setting Up the Virtual Environment

1. Ensure that you have `conda` installed and open anaconda prompt.
2. First, we will change the conda's package-dependency solver to `libmamba`, as it is much faster. Let's first install it by activating the base environment and running
```bash
conda install conda-libmamba-solver
```
3. Deactivate the base enviroment and change the solver:
```bash
conda config --set solver libmamba
```
4. Now its time to install the actual libraries. Navigate the the project directory (with `cd`) and write:
```bash
conda env create --file environment.yml
```
5. Next, to ensure that python 'sees' the .py files in subfolders within project directory, activate the `DSE-18` environment and then run:
```bash
conda develop .
```
6. (Optional) If we add more packages, then run again:
```bash
conda env update --file environment.yml
```
