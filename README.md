# STARK

We present a *Software Tool for the Analysis of Robustness in the unKnown environment* (STARK), our JAVA tool that constitutes the core of our framework for the specification and analysis of properties of distances between the behaviours of systems operating in the presence of uncertainties.
This framework consists in:

  * A model for systems behaviour, called *evolution sequence*, that is defined as a sequence of probability measures over a set of application relevant data (called *data state*). This sequence represents the combined activity of the programs and environment that constitute the considered system.
  * A temporal logic, called *Robustness Temporal Logic* (*RobTL*), for the specification of the desired properties.
  * A *model checking algorithm*, based on stastical inference, for the verification of RobTL specifications.

Hence, STARK includes:

 * A specification language for the programs, the environment, and perturbations.
 * A randomised procedure that, based on simulation, permits the estimation of the evolution sequence of a system $s$, assuming an initial data state $d\_s$. Starting from $d\_s$ we sample N sequences of data states $d\_{(0,j)},...,d\_{(k,j)}$, for $j=1,...,N$. All the data states collected at time i are used to estimate the distribution $S\_{(s,i)}$, i.e., the i-th distribution in the evolution sequence of s.  
 * A procedure that given an evolution sequence permits to sample the effects of a perturbation over it. The same approach used to obtain an estimation of the evolution sequence associated with a given initial data state $d\_s$ can be used to obtain its perturbation. The only difference is that while for evolution sequences the data state obtained at step $i+1$ only depends on the data state at step $i$, here the effect of a perturbation p can be also applied. To guarantee statistical relevance of the collected data, for each sampled data state in the evolution sequence we use and additional number $M$ of samplings to estimate the effects of p over it.
 * A mechanism to estimate the Wasserstein distance between two probability distributions over data states.
 * A function that permits evaluating a distance expression between an evolution sequence and its perturbed variant. A distance expression is evaluated, following a syntax driven procedure, by applying the estimation of the Wasserstein distance considered in the previous step at the involved time steps. 
 * A procedure that checks if a given evolution sequence satisfies a given formula.

Since the procedures outlined above are based on statistical inference, we need to take into account the statistical error when checking the satisfaction of formulae.
Hence, STARK also includes a classical algorithm for the evaluation of confidence intervals in the evaluation of distances, and a three-valued semantics for RobTL specifications, in which the truth value *unknown* is added to true and false.

The [lib](./lib) folder contains all the [Java classes](./lib/src/main/java/it/unicam/quasylab/jspear/) that are necessary to implement all the procedures, mechanisms, and functions described above.

In the [examples/engine](./examples/engine) folder you can find the script [Main.java](./examples/engine/src/main/java/it/unicam/quasylab/jspear/examples/engine/Main.java), that contains various tests, that we used to showcase different features of JSPEAR over the case study of a refrigerated engine system subject to cyber-physical attacks aimed at inflicting overstress of equipment. 

We have used the Python script [plots.py](./plots.py) to draw plots from the CSV files obtained from the JAVA script.

## Download 

To download STARK you have to clone the GitHub project:

```
git clone https://github.com/quasylab/jspear.git
```

Run this command in the folder where you want to download the tool.

## Building STARK

To build STARK you have to execute the following commands from the shell:

```
./gradlew build

./gradlew install
```

## Execute Command Line Interpreter

To run che STARK CLI you have to execute:

```
cd ./cli/build/install/stark 

./bin/stark
```

An example, based on the single vehicle scenario, is available in the folder ```examples\single_vehicle```. 
The ```single_vehicle.jspear``` model can be loaded by using the following command:

```
cd "examples/single_vehicle"

load "single_vehicle.jspec"
```

After that, the command ```formulas``` can be used to view the list of formulas defined in the model. 
Their satisfaction of a formula at a given time step can be checked as follows:

```
check boolean phi_slow_04 at 300

check threevalued always_slow_05 at 0
```

## How to run experiments with Java

You will need
* A text editor or IDE (we used [IntelliJ IDEA](https://www.jetbrains.com/idea/))
* A JAVA Development Kit (JDK). We used version 18. To ensure compatibility with Gradle, please use any version between 8 and 18.
* A [Gradle](https://docs.gradle.org/current/userguide/userguide.html) distribution. We used version 7.4.2.

Firstly, you need to review the files in the project directory by executing:

settings.gradle

Then, you simply need to run the Main.java file:

java Main

To obtain the plots, Python3 (>= 3.9) is needed. Moreover, the following Python packages must be available:

    numpy >= 1.18.4
    scipy >= 1.4.1
    matplotlib >= 3.0.3

If all the needed packages are installed in your system, you have to execute:

python plots.py

after you have obtained the CSV files generated from the JAVA script.
