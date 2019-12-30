# IfaThesis

A Latex template that can be used to prepare theses of various types (master thesis, diploma thesis, dissertation, etc.) at the [Institute of Automation of TU Dresden](http://www.et.tu-dresden.de/ifa/).

**Note**: Before starting to write your own thesis, carefully read the [official guideline of the Institute of Automation](http://www.et.tu-dresden.de/ifa/index.php?id=330) and all linked documents!

## Corporate Design Note
This template is not compliant with the corporate design of TU Dresden. 
Reasons: The CD-Font openSans is only used in the logo header, but not in the text body because sans fonts are not suitable for large text bodies, especially not in thesises.
Placing the institute logo in the header is not suggested. However, if needed, both settings can be overridden by the user.

## License
The content of this repository is made available under the Eclipse Public License v1.0 (see [LICENSE](/LICENSE) file), except logo files (IfA_Logo_SW.pdf, TU_Logo_SW.pdf etc.) that are property of Technische Universität Dresden instution or its subordinate institutions. The use of such protected files is allowed for members or students of Technische Universität Dresden only.

## Usage

The repository not only contains the basic template but also a sample document ([example.tex](/example.tex)) that should provide a good starting point for your own thesis. It explains the possible configuration options, provided commands, and basic usage hints. 

## Setup

There are multiple ways to setup this thesis project for developing your own thesis:

1. **Forking this project** into a new personal thesis project, cloning the new repository into a local working copy. 
   This is probably a good solution, but the remaining forking relation is only useful, if this thesis project shall be further developed.
   For personal thesis projects, no merge request can be applied to this thesis template project.
   Thus, the forking relation should be removed.
   **Note, the template branch is deprecated, see Contributing guide** Updates however, from this project can only be received when this project is added as a remote to the local working copy of the newly forked project.
   In this case, updates should be received only from the `template` branch that is cleaned from the sample files.
2. **Clone this project (Side-by-Side installation)** into a local working copy. Create a new seperate Latex-project in a folder nearby (not within, since the versioning cannot be separated this way). 
   In the preamlbe your main Latex-file, use the command `\documentclass[...]{../ifathesis/ifathesis.cls}`. 
   This way, the versioning for the original thesis template project and the custom thesis project can be separated, updates to the template project will be received, and the example files are constantly available.

**Important:**
1. You should use Git for the versioning on the one hand and on the other hand a GitLab/GitHub-Server Repository for synchonizing the project between multiple devices and for maintaining a backup automatically.
2. Only track files that are not generated during Latex build with the versioning tool (Git). 
   Do not checkin Latex temporary files or the generated output PDF, but check all files in, that are required for a Latex build, i.e. everything that is required to perform the command `pdflatex <main/thesis/etc.>.tex` should be checked-in into verisioning, nothing less, nothing more.
   Unless using setup method 2, e.g. the `ifthesis.cls` and helper files such as `packages.tex`, logo files, etc. should be checked in.
3. Snippet [Snippet 1](https://git.agtele.eats.et.tu-dresden.de/snippets/1) provides a good .gitignore list, that should be added to the thesis project (copy over the version that is cloned with this thesis template project, or on windows create the file gitignore.txt and use the command `mv gitignore.text .gitignore` in order to rename the file).
   This file helps your local git client to track only files, that are necessary to be tracked.
4. For the use of Git, and for other software-related stuff, [Snippet 3](https://git.agtele.eats.et.tu-dresden.de/snippets/3) provides a lecture script for practical aspectes of software development including a short introduction into Git. 

## Contributing

Please see the [contribution guide](/CONTRIBUTING.md).