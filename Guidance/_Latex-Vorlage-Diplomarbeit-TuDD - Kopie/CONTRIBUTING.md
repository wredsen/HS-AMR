# deprecated

The following Contributing Guide is deprecated. 
The two-branch model was intended to be used in order to provide the example document based on the ifathesis class (master) 
and to provide an update source (template) for the class file alone that can be incorporated as a remote in derived thesis project.
This, however, makes the project harder to maintain while there is a better installation option available (Side-by-Side), see Readme.

# Contributing

The repository contains two branches: 'template' and 'master'.

The 'template' branch contains the main files that are necessary for creating a 
thesis (mainly the custom LaTeX class and a set of packages) whereas the
'master' branch contains all contents of the 'template' branch as well as a 
sample project illustrating the use of the LaTeX class.

This separation allows to merge the 'template' branch into ones own thesis
repository without cluttering it with the sample project.

IMPORTANT:

As a result of this structure, all development related to the LaTeX class should
ONLY take place in the 'template' branch!!! It can afterwards be merged into the
'master' branch. Development related to the sample project has in contrast to
take place in the 'master' branch.