FIRM-GDTL
---------

The project implements motion planning for uncertain systems with rich specifications that capture both temporal and uncertainty constraints.
The solution is based on the sampling-based planner FIRM from [1], and automata-based methods.

If you use this work in your research, please cite the following:

<b>Journal paper:</b>

In preparation:
Kevin Leahy*, Eric Cristofalo*, Cristian Ioan Vasile*, Austin Jones, Eduardo Montijano, Mac Schwager, and Calin, Belta. <i>Control in Belief Space with Temporal Logic Specifications using Vision-based Localization</i>.
(*contributed equally)

<b>Control policy synthesis methods (theory):</b>

<i>Control in Belief Space with Temporal Logic Specifications</i>. Cristian Ioan Vasile, Kevin Leahy, Eric Cristofalo, Austin Jones, Mac Schwager, and Calin, Belta. IEEE Conference on Decision and Control (CDC), Las Vegas, NV, USA, December 2016.

bibtex:

<code>
@conference{VaLeCrJoScBe-CDC-2016,
    author    = {Cristian Ioan, Vasile and  Kevin, Leahy and Eric, Cristofalo and Austin, Jones and Mac, Schwager and Calin, Belta},
    title     = {{Control in Belief Space with Temporal Logic Specifications}},
    booktitle = {IEEE Conference on Decision and Control (CDC)},
    pages     = {--},
    month     = {December},
    address   = {Las Vegas, NV, USA},
    year      = {2016},
}
</code>

<b>End-to-end framework vision-based framework (experimental):</b>

<i>Vision-based Mobile Sensing for GPS-deprived Control with Temporal Logic Specifications</i>. Eric Cristofalo, Kevin Leahy, Cristian Ioan Vasile, Eduardo Montijano, Mac Schwager, and Calin, Belta. International Symposium on Experimental Robotics (ISER), Tokyo, Japan, October 2016.

bibtex:

<code>
@conference{CrLeVaMoScBe-ISER-2016,
    author    = {Eric, Cristofalo and Kevin, Leahy and Cristian Ioan, Vasile and Eduardo, Montijano and Mac, Schwager and Calin, Belta},
    title     = {{Vision-based Mobile Sensing for GPS-deprived Control with Temporal Logic Specifications}},
    booktitle = {International Symposium on Experimental Robotics (ISER)},
    pages     = {--},
    month     = {October},
    address   = {Tokyo, Japan},
    year      = {2016},
}
</code>


Setup
-----

1) Install dependencies:

<code>
pip install numpy scipy networkx PyYAMP antlr4-python2-runtime
</code>

2) Get GDTL-FIRM source

<code>
git clone https://github.com/wasserfeder/gdtl-firm.git

cd gdtl-firm/src
</code>

2a) [Temporary step] Switch to the development branch

<code>
git checkout dev-mars-fsa
</code>

3) Get LOMAP and install LOMAP dependencies

<code>
git clone https://github.com/wasserfeder/lomap.git lomap
</code>

3a) Install Spot: Follow the instructions from https://spot.lrde.epita.fr/install.html. If you install from source, don't forget to make the programs visible from the PATH environmental variable.

3b) Install ltl2dstar: Download the program from http://www.ltl2dstar.de/, and follow the instructions from the README file. Don't forget to make the programs visible from the PATH environmental variable.

Mission specification
---------------------

There are two files that need to be created:
1) A mission specification file (YAML) that defines the environment, mission specification formula, the file containing the predicates, vehicle, planner, and simulation parameters. Example file: `src/data/mars/mission.yaml`
2) A python file containing the predicates. Example file: `src/data/mars/predicates.py`

Run
---

To run the method, the `main.py` is executed. A typical example is from the root folder (`gdtl-firm`):

<code>
python src/main.py -o src/data/mars -l src/data/mars/mars.log -v data/mars/mission.yaml
</code>

where `src/data/mars` is the output directory (`-o dir`) used to store generated files, `src/data/mars/mars.log` is the logfile (`-l logfile`), `-v` sets the verbose flag, and `data/mars/mission.yaml` is the mandatory mission yaml file.

For a detailed description of the parameters, execute the script with the '--help' argument.

<code>
python src/main.py --help
</code>

Generate the GDTL lexer and parser
----------------------------------

There are two ways to generate the lexer and parser for GDTL from the grammar file `GDTL.g4`.
NOTE: Make sure that the antlr jar is the same version of the python antlr4 runtime version. To check the version of the python antlr4 runtime run:

pip list | grep antlr4-python2-runtime

The version next to the package should match the jar file in `src/gdtl/lib`. If it does not, then download the matching version from http://www.antlr.org/download.html

<code>
cd src/gdtl/lib

wget -v http://www.antlr.org/download/antlr-4.x-complete.jar
</code>

1) Using ant:

<code>
cd src/gdtl/

ant
</code>

NOTE: If you changed the jar file, then you will need to edit the `build.xml` file, and change the `antlr` property (line 22) to point to the new jar file.

2) Using java:

<code>
cd src/gdtl/

java -cp '$CLASSPATH:lib/antlr-4.7-complete.jar' org.antlr.v4.Tool -visitor -Dlanguage=Python2 GDTL.g4

rm -vfr *.tokens
</code>

Code
----

- The main script is `main.py` that sets up the planner, robot models (motion and observation models), and executes the off-line and on-line (deployment) policies generated by the planner. It also sets up logging.
- The <b>firm</b> package contains an implementation of FIRM, filters, controllers, motion and observation models.
- The <b>gdtl</b> package contains the ANTLR4 grammar for GDTL, and the API to evaluate GDTL formulae using pre-definite and provided predicates.
